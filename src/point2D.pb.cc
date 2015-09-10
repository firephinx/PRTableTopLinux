// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: point2D.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "point2D.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)

namespace personalRobotics {

namespace {

const ::google::protobuf::Descriptor* Point2D_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  Point2D_reflection_ = NULL;

}  // namespace


void protobuf_AssignDesc_point2D_2eproto() {
  protobuf_AddDesc_point2D_2eproto();
  const ::google::protobuf::FileDescriptor* file =
    ::google::protobuf::DescriptorPool::generated_pool()->FindFileByName(
      "point2D.proto");
  GOOGLE_CHECK(file != NULL);
  Point2D_descriptor_ = file->message_type(0);
  static const int Point2D_offsets_[2] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Point2D, x_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Point2D, y_),
  };
  Point2D_reflection_ =
    new ::google::protobuf::internal::GeneratedMessageReflection(
      Point2D_descriptor_,
      Point2D::default_instance_,
      Point2D_offsets_,
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Point2D, _has_bits_[0]),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Point2D, _unknown_fields_),
      -1,
      ::google::protobuf::DescriptorPool::generated_pool(),
      ::google::protobuf::MessageFactory::generated_factory(),
      sizeof(Point2D));
}

namespace {

GOOGLE_PROTOBUF_DECLARE_ONCE(protobuf_AssignDescriptors_once_);
inline void protobuf_AssignDescriptorsOnce() {
  ::google::protobuf::GoogleOnceInit(&protobuf_AssignDescriptors_once_,
                 &protobuf_AssignDesc_point2D_2eproto);
}

void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
    Point2D_descriptor_, &Point2D::default_instance());
}

}  // namespace

void protobuf_ShutdownFile_point2D_2eproto() {
  delete Point2D::default_instance_;
  delete Point2D_reflection_;
}

void protobuf_AddDesc_point2D_2eproto() {
  static bool already_here = false;
  if (already_here) return;
  already_here = true;
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
    "\n\rpoint2D.proto\022\020personalRobotics\"%\n\007Poi"
    "nt2D\022\014\n\001x\030\001 \002(\002:\0010\022\014\n\001y\030\002 \002(\002:\0010", 72);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "point2D.proto", &protobuf_RegisterTypes);
  Point2D::default_instance_ = new Point2D();
  Point2D::default_instance_->InitAsDefaultInstance();
  ::google::protobuf::internal::OnShutdown(&protobuf_ShutdownFile_point2D_2eproto);
}

// Force AddDescriptors() to be called at static initialization time.
struct StaticDescriptorInitializer_point2D_2eproto {
  StaticDescriptorInitializer_point2D_2eproto() {
    protobuf_AddDesc_point2D_2eproto();
  }
} static_descriptor_initializer_point2D_2eproto_;

// ===================================================================

#ifndef _MSC_VER
const int Point2D::kXFieldNumber;
const int Point2D::kYFieldNumber;
#endif  // !_MSC_VER

Point2D::Point2D()
  : ::google::protobuf::Message() {
  SharedCtor();
  // @@protoc_insertion_point(constructor:personalRobotics.Point2D)
}

void Point2D::InitAsDefaultInstance() {
}

Point2D::Point2D(const Point2D& from)
  : ::google::protobuf::Message() {
  SharedCtor();
  MergeFrom(from);
  // @@protoc_insertion_point(copy_constructor:personalRobotics.Point2D)
}

void Point2D::SharedCtor() {
  _cached_size_ = 0;
  x_ = 0;
  y_ = 0;
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

Point2D::~Point2D() {
  // @@protoc_insertion_point(destructor:personalRobotics.Point2D)
  SharedDtor();
}

void Point2D::SharedDtor() {
  if (this != default_instance_) {
  }
}

void Point2D::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* Point2D::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return Point2D_descriptor_;
}

const Point2D& Point2D::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_point2D_2eproto();
  return *default_instance_;
}

Point2D* Point2D::default_instance_ = NULL;

Point2D* Point2D::New() const {
  return new Point2D;
}

void Point2D::Clear() {
#define OFFSET_OF_FIELD_(f) (reinterpret_cast<char*>(      \
  &reinterpret_cast<Point2D*>(16)->f) - \
   reinterpret_cast<char*>(16))

#define ZR_(first, last) do {                              \
    size_t f = OFFSET_OF_FIELD_(first);                    \
    size_t n = OFFSET_OF_FIELD_(last) - f + sizeof(last);  \
    ::memset(&first, 0, n);                                \
  } while (0)

  ZR_(x_, y_);

#undef OFFSET_OF_FIELD_
#undef ZR_

  ::memset(_has_bits_, 0, sizeof(_has_bits_));
  mutable_unknown_fields()->Clear();
}

bool Point2D::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:personalRobotics.Point2D)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoff(127);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // required float x = 1 [default = 0];
      case 1: {
        if (tag == 13) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &x_)));
          set_has_x();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(21)) goto parse_y;
        break;
      }

      // required float y = 2 [default = 0];
      case 2: {
        if (tag == 21) {
         parse_y:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &y_)));
          set_has_y();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectAtEnd()) goto success;
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0 ||
            ::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:personalRobotics.Point2D)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:personalRobotics.Point2D)
  return false;
#undef DO_
}

void Point2D::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:personalRobotics.Point2D)
  // required float x = 1 [default = 0];
  if (has_x()) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(1, this->x(), output);
  }

  // required float y = 2 [default = 0];
  if (has_y()) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(2, this->y(), output);
  }

  if (!unknown_fields().empty()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:personalRobotics.Point2D)
}

::google::protobuf::uint8* Point2D::SerializeWithCachedSizesToArray(
    ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:personalRobotics.Point2D)
  // required float x = 1 [default = 0];
  if (has_x()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(1, this->x(), target);
  }

  // required float y = 2 [default = 0];
  if (has_y()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(2, this->y(), target);
  }

  if (!unknown_fields().empty()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:personalRobotics.Point2D)
  return target;
}

int Point2D::ByteSize() const {
  int total_size = 0;

  if (_has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    // required float x = 1 [default = 0];
    if (has_x()) {
      total_size += 1 + 4;
    }

    // required float y = 2 [default = 0];
    if (has_y()) {
      total_size += 1 + 4;
    }

  }
  if (!unknown_fields().empty()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        unknown_fields());
  }
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = total_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void Point2D::MergeFrom(const ::google::protobuf::Message& from) {
  GOOGLE_CHECK_NE(&from, this);
  const Point2D* source =
    ::google::protobuf::internal::dynamic_cast_if_available<const Point2D*>(
      &from);
  if (source == NULL) {
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
    MergeFrom(*source);
  }
}

void Point2D::MergeFrom(const Point2D& from) {
  GOOGLE_CHECK_NE(&from, this);
  if (from._has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (from.has_x()) {
      set_x(from.x());
    }
    if (from.has_y()) {
      set_y(from.y());
    }
  }
  mutable_unknown_fields()->MergeFrom(from.unknown_fields());
}

void Point2D::CopyFrom(const ::google::protobuf::Message& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Point2D::CopyFrom(const Point2D& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Point2D::IsInitialized() const {
  if ((_has_bits_[0] & 0x00000003) != 0x00000003) return false;

  return true;
}

void Point2D::Swap(Point2D* other) {
  if (other != this) {
    std::swap(x_, other->x_);
    std::swap(y_, other->y_);
    std::swap(_has_bits_[0], other->_has_bits_[0]);
    _unknown_fields_.Swap(&other->_unknown_fields_);
    std::swap(_cached_size_, other->_cached_size_);
  }
}

::google::protobuf::Metadata Point2D::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = Point2D_descriptor_;
  metadata.reflection = Point2D_reflection_;
  return metadata;
}


// @@protoc_insertion_point(namespace_scope)

}  // namespace personalRobotics

// @@protoc_insertion_point(global_scope)