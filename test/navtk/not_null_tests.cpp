#include <memory>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>
#include <error_mode_assert.hpp>

#include <navtk/not_null.hpp>

using navtk::make_not_null;
using navtk::not_null;
using std::shared_ptr;

struct Object {
	int data;

	virtual ~Object() = default;
	Object(int data) : data(data) {}
};
struct ChildObject : public Object {
	ChildObject(int data) : Object(data) {}
};

template <typename T>
struct CustomPtr {
public:
	T* get() const { return ptr; }

	inline T& operator*() { return *ptr; }
	inline T* operator->() { return ptr; }
	inline bool operator==(std::nullptr_t) const { return ptr == nullptr; }

public:
	T* ptr;
};

template <typename T, typename U>
bool operator==(const CustomPtr<T> p_t, const CustomPtr<U> p_u) {
	return p_t.get() == p_u.get();
}



struct NotNullTests : public ::testing::Test {
	Object raw_obj{1};
	Object shared_obj{2};
	Object custom_obj{3};
	Object* raw;
	shared_ptr<Object> shared;
	CustomPtr<Object> custom;

	void SetUp() override {
		raw    = &raw_obj;
		shared = std::make_shared<Object>(shared_obj);
		custom = {&custom_obj};
	}
};

TEST_F(NotNullTests, LValueConstructAndUse) {
	not_null<Object*> raw_test(raw);
	not_null<shared_ptr<Object>> shared_test(shared);
	not_null<CustomPtr<Object>> custom_test(custom);

	// Assert underlying pointers are still non-null
	ASSERT_NO_THROW(raw_test.get());
	ASSERT_NO_THROW(shared_test.get());
	ASSERT_NO_THROW(custom_test.get());

	// Check get() returns underlying pointer
	EXPECT_EQ(raw, raw_test.get());
	EXPECT_EQ(shared, shared_test.get());
	EXPECT_EQ(custom, custom_test.get());

	// Check member dereference pass through
	EXPECT_EQ(raw->data, raw_test->data);
	EXPECT_EQ(shared->data, shared_test->data);
	EXPECT_EQ(custom->data, custom_test->data);

	// Check dereference pass through
	Object raw_obj    = *raw_test;
	Object shared_obj = *shared_test;
	Object custom_obj = *custom_test;
	EXPECT_EQ((*raw).data, raw_obj.data);
	EXPECT_EQ((*shared).data, shared_obj.data);
	EXPECT_EQ((*custom).data, custom_obj.data);
}

TEST_F(NotNullTests, RValueConstructAndUse) {
	not_null<Object*> raw_test(&raw_obj);

	// Assert underlying pointers are still non-null
	ASSERT_NO_THROW(raw_test.get());

	// Check get() returns underlying pointer
	EXPECT_EQ(raw, raw_test.get());

	// Check member dereference pass through
	EXPECT_EQ(raw->data, raw_test->data);

	// Check dereference pass through
	Object raw_obj = *raw_test;
	EXPECT_EQ((*raw).data, raw_obj.data);
}

TEST_F(NotNullTests, RValueConvertConstruct) {
	ChildObject o(5);
	not_null<Object*> o_test(&o);
	EXPECT_EQ(o.data, o_test->data);
}

TEST_F(NotNullTests, ImplicitCast) {
	not_null<Object*> raw_test                    = raw;
	not_null<shared_ptr<Object>> shared_test      = shared;
	not_null<shared_ptr<Object>> make_shared_test = std::make_shared<Object>(shared_obj);
	not_null<CustomPtr<Object>> custom_test       = custom;

	EXPECT_EQ(raw, raw_test.get());
	EXPECT_EQ(shared, shared_test.get());
	EXPECT_EQ(shared->data, make_shared_test->data);
	EXPECT_EQ(custom, custom_test.get());
}

TEST_F(NotNullTests, HelperConstruct) {
	auto ptr = navtk::make_not_null<Object*>(&raw_obj);
	EXPECT_EQ(ptr->data, raw_obj.data);
}

TEST_F(NotNullTests, CopyConstruct) {
	not_null<Object*> raw_test(raw);
	not_null<Object*> raw_copy(raw_test);
	EXPECT_EQ(raw, raw_copy);
}

TEST_F(NotNullTests, ConvertCopyConstruct) {
	ChildObject o(5);
	not_null<ChildObject*> o_test(&o);
	not_null<Object*> o_copy(o_test);
	EXPECT_EQ(o_test, o_copy);
}

TEST_F(NotNullTests, BadConstruct) {
	CustomPtr<Object> bad{nullptr};
	EXPECT_UB_OR_DIE(
	    not_null<CustomPtr<Object>>(bad), "Pointer must be non-null.", std::invalid_argument);
}

TEST_F(NotNullTests, PointerComparison) {
	int a = 1, b = 2;
	not_null<int*> p0(&a);
	not_null<const int*> p1(&b);

	EXPECT_TRUE(p0 == not_null<int*>(&a));
	EXPECT_FALSE(p0 == p1);
	EXPECT_FALSE(p1 == p0);

	EXPECT_FALSE(p0 != not_null<int*>(&a));
	EXPECT_TRUE(p0 != p1);
	EXPECT_TRUE(p1 != p0);

	EXPECT_FALSE(p0 < not_null<int*>(&a));
	EXPECT_TRUE((p0 < p1) == (&a < &b));
	EXPECT_TRUE((p0 < p1) == (&a < &b));

	EXPECT_FALSE(p0 > not_null<int*>(&a));
	EXPECT_TRUE((p0 > p1) == (&a > &b));
	EXPECT_TRUE((p0 > p1) == (&a > &b));

	EXPECT_TRUE(p0 <= not_null<int*>(&a));
	EXPECT_TRUE((p0 <= p1) == (&a <= &b));
	EXPECT_TRUE((p0 <= p1) == (&a <= &b));

	EXPECT_TRUE(p0 >= not_null<int*>(&a));
	EXPECT_TRUE((p0 >= p1) == (&a >= &b));
	EXPECT_TRUE((p0 >= p1) == (&a >= &b));
}

TEST_F(NotNullTests, DynamicCastOverload) {
	not_null<shared_ptr<Object>> o = std::make_shared<ChildObject>(5);
	EXPECT_NE(std::dynamic_pointer_cast<ChildObject>(o), nullptr);
}