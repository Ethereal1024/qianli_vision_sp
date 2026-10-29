#include "simg.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstring>

namespace io::sim
{

std::string shm_name(const std::string & path)
{
  const std::string prefix = "/dev/shm/";
  std::string name = path;
  if (name.compare(0, prefix.size(), prefix) == 0) name = name.substr(prefix.size() - 1);
  if (name.empty()) return name;
  if (name[0] != '/') name = "/" + name;
  return name;
}

ShmChannel::~ShmChannel() { close(); }

uint32_t ShmChannel::load_u32(size_t off) const
{
  return __atomic_load_n(reinterpret_cast<const uint32_t *>(base_ + off), __ATOMIC_ACQUIRE);
}

uint64_t ShmChannel::load_u64(size_t off) const
{
  return __atomic_load_n(reinterpret_cast<const uint64_t *>(base_ + off), __ATOMIC_ACQUIRE);
}

void ShmChannel::store_u32(size_t off, uint32_t value)
{
  __atomic_store_n(reinterpret_cast<uint32_t *>(base_ + off), value, __ATOMIC_RELAXED);
}

void ShmChannel::store_u64(size_t off, uint64_t value)
{
  __atomic_store_n(reinterpret_cast<uint64_t *>(base_ + off), value, __ATOMIC_RELAXED);
}

uint32_t ShmChannel::slot_count() const { return base_ ? load_u32(SIMG_OFF_SLOT_COUNT) : 0; }

uint32_t ShmChannel::slot_size() const { return base_ ? load_u32(SIMG_OFF_SLOT_SIZE) : 0; }

uint64_t ShmChannel::generation() const { return base_ ? load_u64(SIMG_OFF_GENERATION) : 0; }

void ShmChannel::close()
{
  if (base_) {
    munmap(base_, size_);
    base_ = nullptr;
  }
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  size_ = 0;
  last_generation_ = 0;
  path_.clear();
}

bool ShmChannel::map_existing(size_t bytes, bool writable)
{
  int flags = writable ? (PROT_READ | PROT_WRITE) : PROT_READ;
  void * m = mmap(nullptr, bytes, flags, MAP_SHARED, fd_, 0);
  if (m == MAP_FAILED) {
    base_ = nullptr;
    return false;
  }
  base_ = static_cast<uint8_t *>(m);
  size_ = bytes;
  return true;
}

bool ShmChannel::open_reader(const std::string & path)
{
  close();
  path_ = shm_name(path);

  fd_ = shm_open(path_.c_str(), O_RDONLY, 0666);
  if (fd_ < 0) return false;

  struct stat st;
  if (fstat(fd_, &st) != 0 || static_cast<size_t>(st.st_size) < SIMG_HEADER) {
    close();
    return false;
  }

  if (!map_existing(static_cast<size_t>(st.st_size), false)) {
    close();
    return false;
  }

  if (load_u32(SIMG_OFF_MAGIC) != SIMG_MAGIC || load_u32(SIMG_OFF_VERSION) != SIMG_VERSION) {
    close();
    return false;
  }
  return true;
}

bool ShmChannel::open_writer(const std::string & path, uint32_t slot_count, uint32_t slot_size)
{
  close();
  if (slot_count == 0 || slot_size == 0) return false;
  path_ = shm_name(path);

  fd_ = shm_open(path_.c_str(), O_CREAT | O_RDWR, 0666);
  if (fd_ < 0) return false;

  const size_t total = SIMG_HEADER + static_cast<size_t>(slot_count) * slot_size;
  if (ftruncate(fd_, static_cast<off_t>(total)) != 0) {
    close();
    return false;
  }
  if (!map_existing(total, true)) {
    close();
    return false;
  }

  store_u32(SIMG_OFF_MAGIC, 0u);
  store_u32(SIMG_OFF_VERSION, SIMG_VERSION);
  store_u32(SIMG_OFF_SLOT_COUNT, slot_count);
  store_u32(SIMG_OFF_SLOT_SIZE, slot_size);
  store_u32(SIMG_OFF_FLAGS, 0u);
  store_u32(SIMG_OFF_MAGIC + 20, 0u);
  store_u64(SIMG_OFF_GENERATION, 0);
  store_u64(SIMG_OFF_TIMESTAMP, 0);
  std::memset(base_ + SIMG_OFF_META, 0, SIMG_META_SIZE);
  __atomic_thread_fence(__ATOMIC_RELEASE);
  store_u32(SIMG_OFF_MAGIC, SIMG_MAGIC);
  return true;
}

bool ShmChannel::remap()
{
  struct stat st;
  if (fstat(fd_, &st) != 0) return false;
  const size_t bytes = static_cast<size_t>(st.st_size);
  if (bytes < SIMG_HEADER) return false;

  if (base_) {
    munmap(base_, size_);
    base_ = nullptr;
  }
  if (!map_existing(bytes, false)) return false;
  if (load_u32(SIMG_OFF_MAGIC) != SIMG_MAGIC) {
    munmap(base_, size_);
    base_ = nullptr;
    size_ = 0;
    return false;
  }
  return true;
}

int ShmChannel::read(void * out, size_t capacity, uint64_t * timestamp_ns)
{
  if (fd_ < 0 || !out) return -1;

  struct stat st;
  if (fstat(fd_, &st) != 0) return -1;
  if (static_cast<size_t>(st.st_size) != size_) {
    remap();
    return 0;
  }

  const uint32_t count = slot_count();
  const uint32_t unit = slot_size();
  if (count == 0 || unit == 0) return -2;

  const uint64_t gen = load_u64(SIMG_OFF_GENERATION);
  if (gen == 0 || gen == last_generation_) return 0;
  if (capacity < unit) return -3;

  const uint32_t slot = static_cast<uint32_t>((gen - 1) % count);
  std::memcpy(out, base_ + SIMG_HEADER + static_cast<size_t>(slot) * unit, unit);
  last_generation_ = gen;
  __atomic_thread_fence(__ATOMIC_ACQUIRE);

  if (timestamp_ns) *timestamp_ns = load_u64(SIMG_OFF_TIMESTAMP);
  return static_cast<int>(unit);
}

bool ShmChannel::meta(void * out, size_t * out_len) const
{
  if (!base_ || !out || !out_len) return false;
  size_t n = *out_len < SIMG_META_SIZE ? *out_len : SIMG_META_SIZE;
  std::memcpy(out, base_ + SIMG_OFF_META, n);
  *out_len = n;
  return true;
}

bool ShmChannel::write(const void * data, size_t len)
{
  if (!base_ || !data) return false;
  const uint32_t count = slot_count();
  const uint32_t unit = slot_size();
  if (count == 0 || unit == 0) return false;
  if (len < unit) return false;

  const uint64_t gen = load_u64(SIMG_OFF_GENERATION);
  const uint32_t slot = static_cast<uint32_t>(gen % count);
  std::memcpy(base_ + SIMG_HEADER + static_cast<size_t>(slot) * unit, data, unit);

  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  const uint64_t now = static_cast<uint64_t>(ts.tv_sec) * 1000000000ull + ts.tv_nsec;
  store_u64(SIMG_OFF_TIMESTAMP, now);
  __atomic_thread_fence(__ATOMIC_RELEASE);
  store_u64(SIMG_OFF_GENERATION, gen + 1);
  return true;
}

bool ShmChannel::write_meta(const void * meta, size_t len)
{
  if (!base_ || !meta) return false;
  if (len > SIMG_META_SIZE) return false;
  std::memset(base_ + SIMG_OFF_META, 0, SIMG_META_SIZE);
  std::memcpy(base_ + SIMG_OFF_META, meta, len);
  return true;
}

}  // namespace io::sim
