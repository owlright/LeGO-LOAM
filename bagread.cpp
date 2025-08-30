/**
 * #ROSBAG V2.0
 * <record 1><record 2>....<record N>
 *
 */
#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

using Byte = uint8_t;
using ByteVec = std::vector<Byte>;

enum BagRecordType : uint8_t {
    BagHeader = 0x03,
    Connection = 0x07,
    MessageData = 0x02,
    IndexData = 0x04,
    Chunk = 0x05,
    ChunkInfo = 0x06,
};

uint32_t le32(const Byte* p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

uint64_t le64(const Byte* p)
{
    return (uint64_t)p[0] | ((uint64_t)p[1] << 8) | ((uint64_t)p[2] << 16) | ((uint64_t)p[3] << 24)
        | ((uint64_t)p[4] << 32) | ((uint64_t)p[5] << 40) | ((uint64_t)p[6] << 48) | ((uint64_t)p[7] << 56);
}

bool read_exact(std::istream& in, void* dst, size_t n)
{
    in.read(reinterpret_cast<char*>(dst), static_cast<std::streamsize>(n));
    return (size_t)in.gcount() == n;
}

std::string to_printable(const ByteVec& v, size_t max_len = 128)
{
    std::ostringstream oss;
    size_t n = std::min(v.size(), max_len);
    bool all_printable = true;
    for (size_t i = 0; i < n; ++i) {
        if (v[i] < 0x20 || v[i] > 0x7e) {
            all_printable = false;
            break;
        }
    }
    if (all_printable) {
        oss << std::string(reinterpret_cast<const char*>(v.data()), n);
        if (v.size() > n) oss << "...";
    } else {
        oss << "hex:";
        for (size_t i = 0; i < n; ++i) {
            oss << std::hex << std::setfill('0') << std::setw(2) << (int)v[i];
            if (i + 1 < n) oss << ' ';
        }
        if (v.size() > n) oss << " ...";
    }
    return oss.str();
}

std::string to_string_raw(const ByteVec& v) { return std::string(reinterpret_cast<const char*>(v.data()), v.size()); }

class RosbagReader {
public:
    struct Header {
        std::unordered_map<std::string, ByteVec> fields;
    };

    struct Record {
        uint64_t offset = 0;
        Header header;
        ByteVec data;
    };

    struct ConnInfo {
        uint32_t id = 0;
        std::string topic;
        std::string type;
        std::string md5sum;
        std::string callerid;
        bool latching = false;
        size_t msg_def_len = 0;
    };

    class MemReader {
    public:
        const Byte* base;
        size_t size;
        size_t pos = 0;

        explicit MemReader(const Byte* b, size_t s) : base(b), size(s) { }

        bool eof() const { return pos >= size; }

        size_t remain() const { return size - pos; }

        void read_bytes(void* dst, size_t n)
        {
            if (remain() < n) throw std::runtime_error("Chunk data truncated.");
            std::memcpy(dst, base + pos, n);
            pos += n;
        }

        Record read_record_in_chunk(RosbagReader* br)
        {
            Record rec;
            Byte buf4[4];
            if (remain() < 4) throw std::runtime_error("EOF in chunk reading header_len.");
            read_bytes(buf4, 4);
            uint32_t header_len = le32(buf4);
            if (remain() < header_len) throw std::runtime_error("EOF in chunk reading header.");
            ByteVec hbytes(header_len);
            read_bytes(hbytes.data(), header_len);
            Header h = br->parse_header_from_buf(hbytes.data(), hbytes.size());
            if (remain() < 4) throw std::runtime_error("EOF in chunk reading data_len.");
            read_bytes(buf4, 4);
            uint32_t data_len = le32(buf4);
            if (remain() < data_len) throw std::runtime_error("EOF in chunk reading data.");
            ByteVec dbytes(data_len);
            if (data_len) read_bytes(dbytes.data(), data_len);
            rec.header = std::move(h);
            rec.data = std::move(dbytes);
            return rec;
        }
    };

    RosbagReader() = default;

    bool open(const std::string& path)
    {
        in_.open(path, std::ios::binary);
        if (!in_) return false;
        char magic[13];
        if (!read_exact(in_, magic, sizeof(magic))) return false;
        std::string header_magic(magic, sizeof(magic));
        if (header_magic.find("#ROSBAG V2.0", 0) != 0) return false;
        std::cout << "[Magic] " << std::string(magic, sizeof(magic)) << "\n";
        return true;
    }

    int parse_and_print()
    {
        try {
            while (true) {
                if (in_.peek() == std::char_traits<char>::eof()) break;
                std::streampos rec_pos = in_.tellg();
                Record rec = read_record();
                uint8_t op = get_u8(rec.header, "op");
                if (op == BagRecordType::BagHeader) {
                    print_bag_header(rec, rec_pos);
                } else if (op == BagRecordType::Chunk) {
                    print_chunk(rec, rec_pos);
                } else if (op == BagRecordType::IndexData) {
                    print_index_data(rec, rec_pos);
                } else if (op == BagRecordType::ChunkInfo) {
                    print_chunk_info(rec, rec_pos);
                } else if (op == BagRecordType::Connection) {
                    print_connection_top(rec, rec_pos);
                } else if (op == BagRecordType::MessageData) {
                    print_message_data_top(rec, rec_pos);
                } else {
                    std::cout << "[Unknown] @0x" << std::hex << (uint64_t)rec_pos << std::dec << " op=0x" << std::hex
                              << (int)op << std::dec << " header_fields=" << rec.header.fields.size()
                              << " data_len=" << rec.data.size() << "\n";
                }
            }
            std::cout << "[Done] 解析完成。\n";
            return 0;
        } catch (const std::exception& e) {
            std::cerr << "[Error] " << e.what() << "\n";
            return 2;
        }
    }

private:
    std::ifstream in_;
    std::unordered_map<uint32_t, ConnInfo> connections_;

    Header parse_header_from_buf(const Byte* buf, size_t len) const
    {
        Header h;
        size_t off = 0;
        while (off < len) {
            if (len - off < 4) throw std::runtime_error("Header truncated (field_len).");
            uint32_t flen = le32(buf + off);
            off += 4;
            if (flen == 0 || flen > len - off) throw std::runtime_error("Invalid field_len in header.");
            const Byte* field = buf + off;
            off += flen;
            size_t eq = SIZE_MAX;
            for (size_t i = 0; i < flen; ++i) {
                if (field[i] == '=') {
                    eq = i;
                    break;
                }
            }
            if (eq == SIZE_MAX) throw std::runtime_error("Malformed header field (no '=').");
            std::string name(reinterpret_cast<const char*>(field), eq);
            ByteVec value;
            value.insert(value.end(), field + eq + 1, field + flen);
            h.fields.emplace(std::move(name), std::move(value));
        }
        if (off != len) throw std::runtime_error("Header parse overrun.");
        return h;
    }

    uint8_t get_u8(const Header& h, const char* key) const
    {
        auto it = h.fields.find(key);
        if (it == h.fields.end() || it->second.size() != 1)
            throw std::runtime_error(std::string("Missing/invalid '") + key + "'");
        return it->second[0];
    }

    uint32_t get_u32(const Header& h, const char* key) const
    {
        auto it = h.fields.find(key);
        if (it == h.fields.end() || it->second.size() != 4)
            throw std::runtime_error(std::string("Missing/invalid '") + key + "'");
        return le32(it->second.data());
    }

    uint64_t get_u64(const Header& h, const char* key) const
    {
        auto it = h.fields.find(key);
        if (it == h.fields.end() || it->second.size() != 8)
            throw std::runtime_error(std::string("Missing/invalid '") + key + "'");
        return le64(it->second.data());
    }

    std::string get_str(const Header& h, const char* key) const
    {
        auto it = h.fields.find(key);
        if (it == h.fields.end()) throw std::runtime_error(std::string("Missing '") + key + "'");
        return to_string_raw(it->second);
    }

    Record read_record()
    {
        Record rec;
        rec.offset = static_cast<uint64_t>(in_.tellg());
        Byte buf4[4];
        if (!read_exact(in_, buf4, 4)) throw std::runtime_error("Unexpected EOF reading header_len.");
        uint32_t header_len = le32(buf4);
        ByteVec hbytes(header_len);
        if (!read_exact(in_, hbytes.data(), header_len)) throw std::runtime_error("Unexpected EOF reading header.");
        Header h = parse_header_from_buf(hbytes.data(), hbytes.size());
        if (!read_exact(in_, buf4, 4)) throw std::runtime_error("Unexpected EOF reading data_len.");
        uint32_t data_len = le32(buf4);
        ByteVec dbytes(data_len);
        if (data_len > 0 && !read_exact(in_, dbytes.data(), data_len))
            throw std::runtime_error("Unexpected EOF reading data.");
        rec.header = std::move(h);
        rec.data = std::move(dbytes);
        return rec;
    }

    void print_time64(uint64_t t) const
    {
        uint64_t secs = t / 1000000000ULL;
        uint64_t nsec = t % 1000000000ULL;
        std::cout << "time_raw=" << t << " approx=" << secs << "." << std::setw(9) << std::setfill('0') << nsec;
        std::cout << std::setfill(' ');
    }

    void print_preview_hex(const ByteVec& v, size_t max_n = 32) const
    {
        std::cout << "hex[0.." << (v.size() ? std::min(v.size() - 1, max_n - 1) : 0) << "]=";
        size_t n = std::min(v.size(), max_n);
        for (size_t i = 0; i < n; ++i) {
            if (i) std::cout << ' ';
            std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)v[i];
        }
        if (v.size() > n) std::cout << " ...";
        std::cout << std::dec << std::setfill(' ');
    }

    void print_bag_header(const Record& rec, std::streampos rec_pos) const
    {
        std::cout << "[BagHeader] @0x" << std::hex << (uint64_t)rec_pos << std::dec;
        uint64_t index_pos = get_u64(rec.header, "index_pos");
        uint32_t conn_count = get_u32(rec.header, "conn_count");
        uint32_t chunk_count = get_u32(rec.header, "chunk_count");
        std::cout << " index_pos=" << index_pos << " conn_count=" << conn_count << " chunk_count=" << chunk_count
                  << " data_len=" << rec.data.size() << "\n";
    }

    void print_chunk(const Record& rec, std::streampos rec_pos)
    {
        std::string compression = get_str(rec.header, "compression");
        uint32_t uncompressed_size = get_u32(rec.header, "size");
        std::cout << "[Chunk] @0x" << std::hex << (uint64_t)rec_pos << std::dec << " compression=\"" << compression
                  << "\" uncomp_size=" << uncompressed_size << " comp_size=" << rec.data.size() << "\n";
        if (compression == "none") {
            try {
                MemReader mr(rec.data.data(), rec.data.size());
                while (!mr.eof()) {
                    Record inner = mr.read_record_in_chunk(this);
                    uint8_t iop = get_u8(inner.header, "op");
                    if (iop == 0x07) {
                        uint32_t conn_id = get_u32(inner.header, "conn");
                        std::string topic = get_str(inner.header, "topic");
                        Header ch = parse_header_from_buf(inner.data.data(), inner.data.size());
                        ConnInfo info;
                        info.id = conn_id;
                        info.topic = topic;
                        if (ch.fields.count("type")) info.type = get_str(ch, "type");
                        if (ch.fields.count("md5sum")) info.md5sum = get_str(ch, "md5sum");
                        if (ch.fields.count("callerid")) info.callerid = get_str(ch, "callerid");
                        if (ch.fields.count("latching")) info.latching = (get_str(ch, "latching") == "1");
                        if (ch.fields.count("message_definition"))
                            info.msg_def_len = ch.fields.at("message_definition").size();
                        connections_[conn_id] = info;
                        std::cout << "  [Connection] conn=" << conn_id << " topic=\"" << topic << "\"";
                        if (!info.type.empty()) std::cout << " type=" << info.type;
                        if (!info.md5sum.empty()) std::cout << " md5sum=" << info.md5sum;
                        if (!info.callerid.empty()) std::cout << " callerid=" << info.callerid;
                        std::cout << " latching=" << (info.latching ? "true" : "false");
                        std::cout << " msg_def_len=" << info.msg_def_len << "\n";
                    } else if (iop == 0x02) {
                        uint32_t conn_id = get_u32(inner.header, "conn");
                        uint64_t t = get_u64(inner.header, "time");
                        std::cout << "  [MessageData] conn=" << conn_id;
                        auto it = connections_.find(conn_id);
                        if (it != connections_.end()) std::cout << " topic=\"" << it->second.topic << "\"";
                        std::cout << " ";
                        print_time64(t);
                        std::cout << " data_len=" << inner.data.size() << " ";
                        print_preview_hex(inner.data);
                        std::cout << "\n";
                    } else {
                        std::cout << "  [UnknownInChunk] op=0x" << std::hex << (int)iop << std::dec
                                  << " header_fields=" << inner.header.fields.size()
                                  << " data_len=" << inner.data.size() << "\n";
                    }
                }
            } catch (const std::exception& e) {
                std::cout << "  [ChunkParseWarning] " << e.what() << "\n";
            }
        } else {
            std::cout << "  [SkipChunk] compression=\"" << compression << "\"，未解压处理。\n";
        }
    }

    void print_index_data(const Record& rec, std::streampos rec_pos) const
    {
        uint32_t ver = get_u32(rec.header, "ver");
        uint32_t conn_id = get_u32(rec.header, "conn");
        uint32_t count = get_u32(rec.header, "count");
        std::cout << "[IndexData] @0x" << std::hex << (uint64_t)rec_pos << std::dec << " ver=" << ver
                  << " conn=" << conn_id;
        auto it = connections_.find(conn_id);
        if (it != connections_.end()) std::cout << " topic=\"" << it->second.topic << "\"";
        std::cout << " count=" << count;
        if (ver == 1) {
            size_t need = (size_t)count * (8 + 4);
            if (rec.data.size() >= need) {
                if (count > 0) {
                    const Byte* p = rec.data.data();
                    uint64_t t0 = le64(p + 0);
                    uint32_t off0 = le32(p + 8);
                    const Byte* plast = rec.data.data() + (count - 1) * (8 + 4);
                    uint64_t t1 = le64(plast + 0);
                    uint32_t off1 = le32(plast + 8);
                    std::cout << " [t0=";
                    print_time64(t0);
                    std::cout << " off0=" << off0 << "] [tN=";
                    print_time64(t1);
                    std::cout << " offN=" << off1 << "]";
                }
            } else {
                std::cout << " [Warning: data too short for count]";
            }
        }
        std::cout << "\n";
    }

    void print_chunk_info(const Record& rec, std::streampos rec_pos) const
    {
        uint32_t ver = get_u32(rec.header, "ver");
        uint64_t chunk_pos = get_u64(rec.header, "chunk_pos");
        uint64_t start_time = get_u64(rec.header, "start_time");
        uint64_t end_time = get_u64(rec.header, "end_time");
        uint32_t count = get_u32(rec.header, "count");
        std::cout << "[ChunkInfo] @0x" << std::hex << (uint64_t)rec_pos << std::dec << " ver=" << ver
                  << " chunk_pos=" << chunk_pos << " start=";
        print_time64(start_time);
        std::cout << " end=";
        print_time64(end_time);
        std::cout << " conns=" << count;
        if (ver == 1) {
            size_t need = (size_t)count * (4 + 4);
            if (rec.data.size() >= need) {
                const Byte* p = rec.data.data();
                for (uint32_t i = 0; i < count; ++i) {
                    uint32_t cid = le32(p + i * 8 + 0);
                    uint32_t cnum = le32(p + i * 8 + 4);
                    std::cout << " [conn=" << cid;
                    auto it = connections_.find(cid);
                    if (it != connections_.end()) std::cout << " topic=\"" << it->second.topic << "\"";
                    std::cout << " msgs=" << cnum << "]";
                }
            } else {
                std::cout << " [Warning: data too short for count]";
            }
        }
        std::cout << "\n";
    }

    void print_connection_top(const Record& rec, std::streampos rec_pos)
    {
        std::cout << "[Connection@Top] @0x" << std::hex << (uint64_t)rec_pos << std::dec << " ";
        try {
            uint32_t conn_id = get_u32(rec.header, "conn");
            std::string topic = get_str(rec.header, "topic");
            Header ch = parse_header_from_buf(rec.data.data(), rec.data.size());
            ConnInfo info;
            info.id = conn_id;
            info.topic = topic;
            if (ch.fields.count("type")) info.type = get_str(ch, "type");
            if (ch.fields.count("md5sum")) info.md5sum = get_str(ch, "md5sum");
            if (ch.fields.count("callerid")) info.callerid = get_str(ch, "callerid");
            if (ch.fields.count("latching")) info.latching = (get_str(ch, "latching") == "1");
            if (ch.fields.count("message_definition")) info.msg_def_len = ch.fields.at("message_definition").size();
            connections_[conn_id] = info;
            std::cout << "conn=" << conn_id << " topic=\"" << topic << "\" type=" << info.type
                      << " md5sum=" << info.md5sum << " msg_def_len=" << info.msg_def_len << "\n";
        } catch (const std::exception& e) {
            std::cout << "parse_error: " << e.what() << "\n";
        }
    }

    void print_message_data_top(const Record& rec, std::streampos rec_pos) const
    {
        std::cout << "[MessageData@Top] @0x" << std::hex << (uint64_t)rec_pos << std::dec << " ";
        try {
            uint32_t conn_id = get_u32(rec.header, "conn");
            uint64_t t = get_u64(rec.header, "time");
            std::cout << "conn=" << conn_id;
            auto it = connections_.find(conn_id);
            if (it != connections_.end()) std::cout << " topic=\"" << it->second.topic << "\"";
            std::cout << " ";
            print_time64(t);
            std::cout << " data_len=" << rec.data.size() << " ";
            print_preview_hex(rec.data);
            std::cout << "\n";
        } catch (const std::exception& e) {
            std::cout << "parse_error: " << e.what() << "\n";
        }
    }
};

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "用法: " << argv[0] << " <path/to/file.bag>\n";
        return 1;
    }
    RosbagReader reader;
    if (!reader.open(argv[1])) {
        std::cerr << "无法打开或识别文件。\n";
        return 1;
    }
    return reader.parse_and_print();
}
