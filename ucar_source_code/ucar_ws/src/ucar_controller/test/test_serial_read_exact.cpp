#include <gtest/gtest.h>

#include <algorithm>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

#include <ucar_controller/serial_read_exact.h>

namespace
{

class ChunkReader
{
public:
  explicit ChunkReader(const std::vector<std::string>& chunks)
      : chunks_(chunks), index_(0) {}

  size_t operator()(uint8_t* destination, size_t capacity)
  {
    if (index_ >= chunks_.size())
    {
      return 0;
    }
    const std::string& chunk = chunks_[index_++];
    const size_t count = std::min(capacity, chunk.size());
    std::memcpy(destination, chunk.data(), count);
    return count;
  }

private:
  std::vector<std::string> chunks_;
  size_t index_;
};

TEST(SerialReadExactlyTest, JoinsSplitFields)
{
  uint8_t destination[6] = {0};
  ChunkReader reader(std::vector<std::string>{"ab", "c", "def"});

  const ucarController::SerialReadResult result =
      ucarController::serialReadExactly(destination, sizeof(destination), reader);

  EXPECT_TRUE(result.complete());
  EXPECT_EQ(sizeof(destination), result.received);
  EXPECT_EQ(3u, result.read_calls);
  EXPECT_EQ(0, std::memcmp(destination, "abcdef", sizeof(destination)));
}

TEST(SerialReadExactlyTest, ReportsIncompleteFieldAfterNoProgress)
{
  uint8_t destination[6] = {0};
  ChunkReader reader(std::vector<std::string>{"ab"});

  const ucarController::SerialReadResult result =
      ucarController::serialReadExactly(destination, sizeof(destination), reader);

  EXPECT_FALSE(result.complete());
  EXPECT_EQ(2u, result.received);
  EXPECT_EQ(2u, result.read_calls);
}

TEST(SerialReadExactlyTest, RejectsReaderOverrun)
{
  uint8_t destination[2] = {0};
  const ucarController::SerialReadResult result =
      ucarController::serialReadExactly(
          destination, sizeof(destination),
          [](uint8_t*, size_t capacity) { return capacity + 1; });

  EXPECT_FALSE(result.complete());
  EXPECT_EQ(0u, result.received);
  EXPECT_EQ(1u, result.read_calls);
}

TEST(SerialReadExactlyTest, StopsAfterBudgetDespiteContinuedProgress)
{
  uint8_t destination[6] = {0};
  size_t read_calls = 0;
  const ucarController::SerialReadResult result =
      ucarController::serialReadExactly(
          destination, sizeof(destination),
          [&read_calls](uint8_t* target, size_t) {
            target[0] = static_cast<uint8_t>('a' + read_calls);
            ++read_calls;
            return 1u;
          },
          [&read_calls]() { return read_calls < 3; });

  EXPECT_FALSE(result.complete());
  EXPECT_EQ(3u, result.received);
  EXPECT_EQ(3u, result.read_calls);
  EXPECT_EQ(0, std::memcmp(destination, "abc\0\0\0", sizeof(destination)));
}

TEST(SerialReadExactlyTest, DropsTimedOutFieldBeforeFollowingValidField)
{
  uint8_t destination[6];
  std::memset(destination, 0xA5, sizeof(destination));
  ChunkReader scripted_reader(std::vector<std::string>{"ab", "", "uvwxyz"});
  const ucarController::SerialReadResult truncated =
      ucarController::serialReadExactly(destination, sizeof(destination),
                                        std::ref(scripted_reader));

  size_t dispatched_frames = 0;
  if (truncated.complete())
  {
    ++dispatched_frames;
  }
  EXPECT_FALSE(truncated.complete());
  EXPECT_EQ(0, std::memcmp(destination, "ab\0\0\0\0", sizeof(destination)));

  const ucarController::SerialReadResult valid =
      ucarController::serialReadExactly(destination, sizeof(destination),
                                        std::ref(scripted_reader));
  if (valid.complete())
  {
    ++dispatched_frames;
  }

  EXPECT_TRUE(valid.complete());
  EXPECT_EQ(1u, dispatched_frames);
  EXPECT_EQ(0, std::memcmp(destination, "uvwxyz", sizeof(destination)));
}

}  // namespace

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
