#ifndef IO__SIM__SIMG_HPP
#define IO__SIM__SIMG_HPP

#include <cstddef>
#include <cstdint>
#include <string>

namespace io::sim
{

constexpr uint32_t SIMG_MAGIC = 0x53494D47u;
constexpr uint32_t SIMG_VERSION = 1u;
constexpr size_t SIMG_HEADER = 64;
constexpr size_t SIMG_META_SIZE = 16;

enum : size_t
{
  SIMG_OFF_MAGIC = 0,
  SIMG_OFF_VERSION = 4,
  SIMG_OFF_SLOT_COUNT = 8,
  SIMG_OFF_SLOT_SIZE = 12,
  SIMG_OFF_FLAGS = 16,
  SIMG_OFF_GENERATION = 24,
  SIMG_OFF_TIMESTAMP = 32,
  SIMG_OFF_META = 40,
};

std::string shm_name(const std::string & path);

class ShmChannel
{
public:
  ShmChannel() = default;
  ~ShmChannel();

  ShmChannel(const ShmChannel &) = delete;
  ShmChannel & operator=(const ShmChannel &) = delete;

  bool open_reader(const std::string & path);

  bool open_writer(const std::string & path, uint32_t slot_count, uint32_t slot_size);

  bool opened() const { return base_ != nullptr; }
  const std::string & path() const { return path_; }
  uint32_t slot_count() const;
  uint32_t slot_size() const;

  int read(void * out, size_t capacity, uint64_t * timestamp_ns = nullptr);

  bool meta(void * out, size_t * out_len) const;

  bool write(const void * data, size_t len);

  bool write_meta(const void * meta, size_t len);

  uint64_t generation() const;

  void close();

private:
  bool map_existing(size_t bytes, bool writable);
  bool remap();
  uint32_t load_u32(size_t off) const;
  uint64_t load_u64(size_t off) const;
  void store_u32(size_t off, uint32_t value);
  void store_u64(size_t off, uint64_t value);

  int fd_ = -1;
  uint8_t * base_ = nullptr;
  size_t size_ = 0;
  uint64_t last_generation_ = 0;
  std::string path_;
};

}  // namespace io::sim

#endif  // IO__SIM__SIMG_HPP
