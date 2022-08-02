//# This file is a part of toml++ and is subject to the the terms of the MIT license.
//# Copyright (c) Mark Gillard <mark.gillard@outlook.com.au>
//# See https://github.com/marzer/tomlplusplus/blob/master/LICENSE for the full license text.
// SPDX-License-Identifier: MIT

#pragma once
#include "table.h"
#include "array.h"
#include "value.h"

TOML_PUSH_WARNINGS;
TOML_DISABLE_ARITHMETIC_WARNINGS;

TOML_NAMESPACE_START
{
	/// \brief	A view of a node.
	///
	/// \detail A node_view is like a std::optional<toml::node&> (if such a construct were legal), with lots of
	///			toml-specific stuff built-in. It _may_ represent a node, and allows you to do many of the
	///			same operations that you'd do on nodes directly, as well as easily traversing the node tree by creating
	/// 		subviews (via node_view::operator[]). \cpp
	///
	/// auto tbl = toml::parse(R"(
	///
	///		title = "my hardware store"
	///
	///		[[products]]
	///		name = "Hammer"
	///		sku = 738594937
	///		keywords = [ "hammer", "construction", "build" ]
	///
	///		[[products]]
	///		name = "Nail"
	///		sku = 284758393
	///		color = "gray"
	///
	/// )"sv);
	///
	/// std::cout << tbl["title"] << "\n";
	/// std::cout << tbl["products"][0]["name"] << "\n";
	/// std::cout << tbl["products"][0]["keywords"] << "\n";
	/// std::cout << tbl["products"][0]["keywords"][2] << "\n";
	///
	/// tbl["products"][0]["keywords"].as_array()->push_back("heavy");
	/// std::cout << tbl["products"][0]["keywords"] << "\n";
	/// std::cout << "has product[2]: "sv << !!tbl["products"][2] << "\n";
	/// std::cout << "product[2]: "sv << tbl["products"][2] << "\n";
	/// \ecpp
	///
	/// \out
	/// "my hardware store"
	/// "Hammer"
	/// [ "hammer", "construction", "build" ]
	/// "build"
	/// [ "hammer", "construction", "build", "heavy" ]
	/// has product[2]: false
	/// product[2]:
	/// \eout
	template <typename ViewedType>
	class TOML_API TOML_TRIVIAL_ABI node_view
	{
		static_assert(impl::is_one_of<ViewedType, toml::node, const toml::node>,
					  "A toml::node_view<> must wrap toml::node or const toml::node.");

	  public:
		using viewed_type = ViewedType;

	  private:
		template <typename T>
		friend class TOML_NAMESPACE::node_view;

		mutable viewed_type* node_ = nullptr;

		template <typename Func>
		static constexpr bool visit_is_nothrow = noexcept(std::declval<viewed_type*>()->visit(std::declval<Func&&>()));

	  public:
		/// \brief	Constructs an empty node view.
		TOML_NODISCARD_CTOR
		node_view() noexcept = default;

		/// \brief	Constructs node_view of a specific node.
		TOML_NODISCARD_CTOR
		explicit node_view(viewed_type* node) noexcept //
			: node_{ node }
		{}

		/// \brief	Constructs node_view of a specific node.
		TOML_NODISCARD_CTOR
		explicit node_view(viewed_type& node) noexcept //
			: node_{ &node }
		{}

		/// \brief	Copy constructor.
		TOML_NODISCARD_CTOR
		node_view(const node_view&) noexcept = default;

		/// \brief	Copy-assignment operator.
		node_view& operator=(const node_view&) & noexcept = default;

		/// \brief	Move constructor.
		TOML_NODISCARD_CTOR
		node_view(node_view&&) noexcept = default;

		/// \brief	Move-assignment operator.
		node_view& operator=(node_view&&) & noexcept = default;

		/// \brief	Returns true if the view references a node.
		TOML_NODISCARD
		explicit operator bool() const noexcept
		{
			return node_ != nullptr;
		}

		/// \brief	Returns the node that's being referenced by the view.
		TOML_NODISCARD
		viewed_type* node() const noexcept
		{
			return node_;
		}

		/// \name Type checks
		/// @{

		/// \brief	Returns the type identifier for the viewed node.
		TOML_NODISCARD
		node_type type() const noexcept
		{
			return node_ ? node_->type() : node_type::none;
		}

		/// \brief	Returns true if the viewed node is a toml::table.
		TOML_NODISCARD
		bool is_table() const noexcept
		{
			return node_ && node_->is_table();
		}

		/// \brief	Returns true if the viewed node is a toml::array.
		TOML_NODISCARD
		bool is_array() const noexcept
		{
			return node_ && node_->is_array();
		}

		/// \brief	Returns true if the viewed node is a toml::value<>.
		TOML_NODISCARD
		bool is_value() const noexcept
		{
			return node_ && node_->is_value();
		}

		/// \brief	Returns true if the viewed node is a toml::value<string>.
		TOML_NODISCARD
		bool is_string() const noexcept
		{
			return node_ && node_->is_string();
		}

		/// \brief	Returns true if the viewed node is a toml::value<int64_t>.
		TOML_NODISCARD
		bool is_integer() const noexcept
		{
			return node_ && node_->is_integer();
		}

		/// \brief	Returns true if the viewed node is a toml::value<double>.
		TOML_NODISCARD
		bool is_floating_point() const noexcept
		{
			return node_ && node_->is_floating_point();
		}

		/// \brief	Returns true if the viewed node is a toml::value<int64_t> or toml::value<double>.
		TOML_NODISCARD
		bool is_number() const noexcept
		{
			return node_ && node_->is_number();
		}

		/// \brief	Returns true if the viewed node is a toml::value<bool>.
		TOML_NODISCARD
		bool is_boolean() const noexcept
		{
			return node_ && node_->is_boolean();
		}

		/// \brief	Returns true if the viewed node is a toml::value<date>.
		TOML_NODISCARD
		bool is_date() const noexcept
		{
			return node_ && node_->is_date();
		}

		/// \brief	Returns true if the viewed node is a toml::value<time>.
		TOML_NODISCARD
		bool is_time() const noexcept
		{
			return node_ && node_->is_time();
		}

		/// \brief	Returns true if the viewed node is a toml::value<date_time>.
		TOML_NODISCARD
		bool is_date_time() const noexcept
		{
			return node_ && node_->is_date_time();
		}

		/// \brief	Returns true if the viewed node is a toml::array that contains only tables.
		TOML_NODISCARD
		bool is_array_of_tables() const noexcept
		{
			return node_ && node_->is_array_of_tables();
		}

		/// \brief	Checks if this view references a node of a specific type.
		///
		/// \tparam	T	A TOML node or value type.
		///
		/// \returns	Returns true if the viewed node is an instance of the specified type.
		///
		/// \see toml::node::is()
		template <typename T>
		TOML_NODISCARD
		bool is() const noexcept
		{
			return node_ ? node_->template is<T>() : false;
		}

		/// \brief	Checks if the viewed node contains values/elements of only one type.
		///
		/// \detail \cpp
		/// auto cfg = toml::parse("arr = [ 1, 2, 3, 4.0 ]");
		///
		/// toml::node* nonmatch{};
		/// if (cfg["arr"].is_homogeneous(toml::node_type::integer, nonmatch))
		/// 	std::cout << "array was homogeneous"sv << "\n";
		/// else
		/// 	std::cout << "array was not homogeneous!\n"
		/// 	<< "first non-match was a "sv << nonmatch->type() << " at " << nonmatch->source() << "\n";
		/// \ecpp
		///
		/// \out
		/// array was not homogeneous!
		///	first non-match was a floating-point at line 1, column 18
		/// \eout
		///
		/// \param	ntype	A TOML node type. <br>
		/// 				\conditional_return{toml::node_type::none} "is every element the same type?"
		/// 				\conditional_return{Anything else} "is every element one of these?"
		///
		/// \param first_nonmatch	Reference to a pointer in which the address of the first non-matching element
		/// 						will be stored if the return value is false.
		///
		/// \returns	True if the viewed node was homogeneous.
		///
		/// \remarks	Always returns `false` if the view does not reference a node, or if the viewed node is
		/// 			an empty table or array.
		TOML_NODISCARD
		bool is_homogeneous(node_type ntype, viewed_type*& first_nonmatch) const noexcept
		{
			if (!node_)
			{
				first_nonmatch = {};
				return false;
			}
			return node_->is_homogeneous(ntype, first_nonmatch);
		}

		/// \brief	Checks if the viewed node contains values/elements of only one type.
		///
		/// \detail \cpp
		/// auto cfg = toml::parse("arr = [ 1, 2, 3 ]");
		/// std::cout << "homogenous: "sv << cfg["arr"].is_homogeneous(toml::node_type::none) << "\n";
		/// std::cout << "all floats: "sv << cfg["arr"].is_homogeneous(toml::node_type::floating_point) << "\n";
		/// std::cout << "all arrays: "sv << cfg["arr"].is_homogeneous(toml::node_type::array) << "\n";
		/// std::cout << "all ints:   "sv << cfg["arr"].is_homogeneous(toml::node_type::integer) << "\n";
		///
		/// \ecpp
		///
		/// \out
		/// homogeneous: true
		/// all floats:  false
		/// all arrays:  false
		/// all ints:    true
		/// \eout
		///
		/// \param	ntype	A TOML node type. <br>
		/// 				\conditional_return{toml::node_type::none} "is every element the same type?"
		/// 				\conditional_return{Anything else} "is every element one of these?"
		///
		/// \returns	True if the viewed node was homogeneous.
		///
		/// \remarks	Always returns `false` if the view does not reference a node, or if the viewed node is
		/// 			an empty table or array.
		TOML_NODISCARD
		bool is_homogeneous(node_type ntype) const noexcept
		{
			return node_ ? node_->is_homogeneous(ntype) : false;
		}

		/// \brief	Checks if the viewed node contains values/elements of only one type.
		///
		/// \detail \cpp
		/// auto cfg = toml::parse("arr = [ 1, 2, 3 ]");
		/// std::cout << "homogenous:   "sv << cfg["arr"].is_homogeneous() << "\n";
		/// std::cout << "all doubles:  "sv << cfg["arr"].is_homogeneous<double>() << "\n";
		/// std::cout << "all arrays:   "sv << cfg["arr"].is_homogeneous<toml::array>() << "\n";
		/// std::cout << "all integers: "sv << cfg["arr"].is_homogeneous<int64_t>() << "\n";
		///
		/// \ecpp
		///
		/// \out
		/// homogeneous: true
		/// all floats:  false
		/// all arrays:  false
		/// all ints:    true
		/// \eout
		///
		/// \tparam	ElemType	A TOML node or value type. <br>
		/// 					\conditional_return{Left as `void`} "is every element the same type?" <br>
		/// 					\conditional_return{Explicitly specified} "is every element a T?"
		///
		/// \returns	True if the viewed node was homogeneous.
		///
		/// \remarks	Always returns `false` if the view does not reference a node, or if the viewed node is
		/// 			an empty table or array.
		template <typename ElemType = void>
		TOML_NODISCARD
		bool is_homogeneous() const noexcept
		{
			return node_ ? node_->template is_homogeneous<impl::unwrap_node<ElemType>>() : false;
		}

		/// @}

		/// \name Type casts
		/// @{

		/// \brief	Returns a pointer to the viewed node as a toml::table, if it is one.
		TOML_NODISCARD
		auto as_table() const noexcept
		{
			return as<table>();
		}

		/// \brief	Returns a pointer to the viewed node as a toml::array, if it is one.
		TOML_NODISCARD
		auto as_array() const noexcept
		{
			return as<array>();
		}

		/// \brief	Returns a pointer to the viewed node as a toml::value<string>, if it is one.
		TOML_NODISCARD
		auto as_string() const noexcept
		{
			return as<std::string>();
		}

		/// \brief	Returns a pointer to the viewed node as a toml::value<int64_t>, if it is one.
		TOML_NODISCARD
		auto as_integer() const noexcept
		{
			return as<int64_t>();
		}

		/// \brief	Returns a pointer to the viewed node as a toml::value<double>, if it is one.
		TOML_NODISCARD
		auto as_floating_point() const noexcept
		{
			return as<double>();
		}

		/// \brief	Returns a pointer to the viewed node as a toml::value<bool>, if it is one.
		TOML_NODISCARD
		auto as_boolean() const noexcept
		{
			return as<bool>();
		}

		/// \brief	Returns a pointer to the viewed node as a toml::value<date>, if it is one.
		TOML_NODISCARD
		auto as_date() const noexcept
		{
			return as<date>();
		}

		/// \brief	Returns a pointer to the viewed node as a toml::value<time>, if it is one.
		TOML_NODISCARD
		auto as_time() const noexcept
		{
			return as<time>();
		}

		/// \brief	Returns a pointer to the viewed node as a toml::value<date_time>, if it is one.
		TOML_NODISCARD
		auto as_date_time() const noexcept
		{
			return as<date_time>();
		}

		/// \brief	Gets a pointer to the viewed node as a more specific node type.
		///
		/// \tparam	T	The node type or TOML value type to cast to.
		///
		/// \returns	A pointer to the node as the given type, or nullptr if it was a different type.
		///
		/// \see toml::node::as()
		template <typename T>
		TOML_NODISCARD
		auto as() const noexcept
		{
			return node_ ? node_->template as<T>() : nullptr;
		}

		/// @}

		/// \name Value retrieval
		/// @{

		/// \brief	Gets the value contained by the referenced node.
		///
		/// \detail This function has 'exact' retrieval semantics; the only return value types allowed are the
		/// 		TOML native value types, or types that can losslessly represent a native value type (e.g.
		/// 		std::wstring on Windows).
		///
		/// \tparam	T	One of the native TOML value types, or a type capable of losslessly representing one.
		///
		/// \returns	The underlying value if the node was a value of the
		/// 			matching type (or losslessly convertible to it), or an empty optional.
		///
		/// \see node_view::value()
		template <typename T>
		TOML_NODISCARD
		optional<T> value_exact() const noexcept
		{
			if (node_)
				return node_->template value_exact<T>();
			return {};
		}

		TOML_PUSH_WARNINGS;
		TOML_DISABLE_INIT_WARNINGS;

		/// \brief	Gets the value contained by the referenced node.
		///
		/// \detail This function has 'permissive' retrieval semantics; some value types are allowed
		/// 		to convert to others (e.g. retrieving a boolean as an integer), and the specified return value
		/// 		type can be any type where a reasonable conversion from a native TOML value exists
		/// 		(e.g. std::wstring on Windows). If the source value cannot be represented by
		/// 		the destination type, an empty optional is returned. See node::value() for examples.
		///
		/// \tparam	T	One of the native TOML value types, or a type capable of convertible to one.
		///
		/// \returns	The underlying value if the node was a value of the matching type (or convertible to it)
		/// 			and within the range of the output type, or an empty optional.
		///
		/// \note	If you want strict value retrieval semantics that do not allow for any type conversions,
		/// 		use node_view::value_exact() instead.
		///
		/// \see
		/// 	- node_view::value()
		///		- node_view::value_exact()
		template <typename T>
		TOML_NODISCARD
		optional<T> value() const noexcept
		{
			if (node_)
				return node_->template value<T>();
			return {};
		}

		TOML_POP_WARNINGS;

		/// \brief	Gets the raw value contained by the referenced node, or a default.
		///
		/// \tparam	T				Default value type. Must be one of the native TOML value types,
		/// 						or convertible to it.
		/// \param 	default_value	The default value to return if the node wasn't a value, wasn't the
		/// 						correct type, or no conversion was possible.
		///
		/// \returns	The underlying value if the node was a value of the matching type (or convertible to it)
		/// 			and within the range of the output type, or the provided default.
		///
		/// \note	This function has the same permissive retrieval semantics as node::value(). If you want strict
		/// 		value retrieval semantics that do not allow for any type conversions, use node_view::value_exact()
		/// 		instead.
		///
		/// \see
		/// 	- node_view::value()
		///		- node_view::value_exact()
		template <typename T>
		TOML_NODISCARD
		auto value_or(T&& default_value) const noexcept
		{
			using namespace ::toml::impl;

			static_assert(!is_wide_string<T> || TOML_WINDOWS_COMPAT,
						  "Retrieving values as wide-character strings is only "
						  "supported on Windows with TOML_WINDOWS_COMPAT enabled.");

			if constexpr (is_wide_string<T>)
			{
#if TOML_WINDOWS_COMPAT

				if (node_)
					return node_->value_or(static_cast<T&&>(default_value));
				return std::wstring{ static_cast<T&&>(default_value) };

#else

				static_assert(impl::dependent_false<T>, "Evaluated unreachable branch!");

#endif
			}
			else
			{
				using value_type =
					std::conditional_t<std::is_pointer_v<std::decay_t<T>>,
									   std::add_pointer_t<std::add_const_t<std::remove_pointer_t<std::decay_t<T>>>>,
									   std::decay_t<T>>;

				if (node_)
					return node_->value_or(static_cast<T&&>(default_value));
				if constexpr (std::is_pointer_v<value_type>)
					return value_type{ default_value };
				else
					return static_cast<T&&>(default_value);
			}
		}

		/// \brief	Gets a raw reference to the viewed node's underlying data.
		///
		/// \warning This function is dangerous if used carelessly and **WILL** break your code if the
		/// 		 node_view didn't reference a node, or the chosen value type doesn't match the node's
		/// 		 actual type. In debug builds an assertion will fire when invalid accesses are attempted: \cpp
		///
		/// auto tbl = toml::parse(R"(
		///		min = 32
		///		max = 45
		/// )"sv);
		///
		/// int64_t& min_ref = tbl["min"].ref<int64_t>(); // matching type
		/// double& max_ref = tbl["max"].ref<double>();  // mismatched type, hits assert()
		/// int64_t& foo_ref = tbl["foo"].ref<int64_t>(); // nonexistent key, hits assert()
		///
		/// \ecpp
		///
		/// \tparam	T	One of the TOML value types.
		///
		/// \returns	A reference to the underlying data.
		template <typename T>
		TOML_NODISCARD
		decltype(auto) ref() const noexcept
		{
			TOML_ASSERT(node_ && "toml::node_view::ref() called on a node_view that did not reference a node");
			return node_->template ref<impl::unwrap_node<T>>();
		}

		/// @}

		/// \name Visitation
		/// @{

		/// \brief	Invokes a visitor on the viewed node based on its concrete type.
		///
		/// \remarks Has no effect if the view does not reference a node.
		///
		/// \see node::visit()
		template <typename Func>
		decltype(auto) visit(Func&& visitor) const noexcept(visit_is_nothrow<Func&&>)
		{
			using return_type = decltype(node_->visit(static_cast<Func&&>(visitor)));
			if (node_)
				return node_->visit(static_cast<Func&&>(visitor));
			if constexpr (!std::is_void_v<return_type>)
				return return_type{};
		}

		/// @}

		/// \name Equality
		/// @{

		/// \brief	Returns true if the viewed node is a table with the same contents as RHS.
		TOML_NODISCARD
		friend bool operator==(const node_view& lhs, const table& rhs) noexcept
		{
			if (lhs.node_ == &rhs)
				return true;
			const auto tbl = lhs.as<table>();
			return tbl && *tbl == rhs;
		}
		TOML_ASYMMETRICAL_EQUALITY_OPS(const node_view&, const table&, );

		/// \brief	Returns true if the viewed node is an array with the same contents as RHS.
		TOML_NODISCARD
		friend bool operator==(const node_view& lhs, const array& rhs) noexcept
		{
			if (lhs.node_ == &rhs)
				return true;
			const auto arr = lhs.as<array>();
			return arr && *arr == rhs;
		}
		TOML_ASYMMETRICAL_EQUALITY_OPS(const node_view&, const array&, );

		/// \brief	Returns true if the viewed node is a value with the same value as RHS.
		template <typename T>
		TOML_NODISCARD
		friend bool operator==(const node_view& lhs, const toml::value<T>& rhs) noexcept
		{
			if (lhs.node_ == &rhs)
				return true;
			const auto val = lhs.as<T>();
			return val && *val == rhs;
		}
		TOML_ASYMMETRICAL_EQUALITY_OPS(const node_view&, const toml::value<T>&, template <typename T>);

		/// \brief	Returns true if the viewed node is a value with the same value as RHS.
		TOML_CONSTRAINED_TEMPLATE((impl::is_native<T> || impl::is_losslessly_convertible_to_native<T>), typename T)
		TOML_NODISCARD
		friend bool operator==(const node_view& lhs, const T& rhs) noexcept
		{
			static_assert(!impl::is_wide_string<T> || TOML_WINDOWS_COMPAT,
						  "Comparison with wide-character strings is only "
						  "supported on Windows with TOML_WINDOWS_COMPAT enabled.");

			if constexpr (impl::is_wide_string<T>)
			{
#if TOML_WINDOWS_COMPAT
				return lhs == impl::narrow(rhs);
#else
				static_assert(impl::dependent_false<T>, "Evaluated unreachable branch!");
#endif
			}
			else
			{
				const auto val = lhs.as<impl::native_type_of<T>>();
				return val && *val == rhs;
			}
		}
		TOML_ASYMMETRICAL_EQUALITY_OPS(
			const node_view&,
			const T&,
			TOML_CONSTRAINED_TEMPLATE((impl::is_native<T> || impl::is_losslessly_convertible_to_native<T>),
									  typename T));

		/// \brief	Returns true if the viewed node is an array with the same contents as the RHS initializer list.
		template <typename T>
		TOML_NODISCARD
		friend bool operator==(const node_view& lhs, const std::initializer_list<T>& rhs) noexcept
		{
			const auto arr = lhs.as<array>();
			return arr && *arr == rhs;
		}
		TOML_ASYMMETRICAL_EQUALITY_OPS(const node_view&, const std::initializer_list<T>&, template <typename T>);

		/// \brief	Returns true if the viewed node is an array with the same contents as the RHS vector.
		template <typename T>
		TOML_NODISCARD
		friend bool operator==(const node_view& lhs, const std::vector<T>& rhs) noexcept
		{
			const auto arr = lhs.as<array>();
			return arr && *arr == rhs;
		}
		TOML_ASYMMETRICAL_EQUALITY_OPS(const node_view&, const std::vector<T>&, template <typename T>);

		/// @}

		/// \name Subviews
		/// @{

		/// \brief	Returns a view of the selected subnode.
		///
		/// \param 	key	The key of the node to retrieve
		///
		/// \returns	A view of the selected node if this node represented a table and it contained a
		/// 			value at the given key, or an empty view.
		TOML_NODISCARD
		node_view operator[](std::string_view key) const noexcept
		{
			if (auto tbl = this->as_table())
				return node_view{ tbl->get(key) };
			return node_view{ nullptr };
		}

#if TOML_WINDOWS_COMPAT

		/// \brief	Returns a view of the selected subnode.
		///
		/// \availability This overload is only available when #TOML_WINDOWS_COMPAT is enabled.
		///
		/// \param 	key	The key of the node to retrieve
		///
		/// \returns	A view of the selected node if this node represented a table and it contained a
		/// 			value at the given key, or an empty view.
		TOML_NODISCARD
		node_view operator[](std::wstring_view key) const noexcept
		{
			if (auto tbl = this->as_table())
				return node_view{ tbl->get(key) };
			return node_view{ nullptr };
		}

#endif // TOML_WINDOWS_COMPAT

		/// \brief	Returns a view of the selected subnode.
		///
		/// \param 	index The index of the node to retrieve
		///
		/// \returns	A view of the selected node if this node represented an array and it contained a
		/// 			value at the given index, or an empty view.
		TOML_NODISCARD
		node_view operator[](size_t index) const noexcept
		{
			if (auto arr = this->as_array())
				return node_view{ arr->get(index) };
			return node_view{ nullptr };
		}

		/// @}

		template <typename Char, typename T>
		friend std::basic_ostream<Char>& operator<<(std::basic_ostream<Char>&, const node_view<T>&);
	};
	template <typename T>
	node_view(const value<T>&) -> node_view<const node>;
	node_view(const table&)->node_view<const node>;
	node_view(const array&)->node_view<const node>;
	template <typename T>
	node_view(value<T>&) -> node_view<node>;
	node_view(table&)->node_view<node>;
	node_view(array&)->node_view<node>;
	template <typename T>
	node_view(const T*) -> node_view<const node>;
	template <typename T>
	node_view(T*) -> node_view<node>;

	/// \brief	Prints the viewed node out to a stream.
	template <typename Char, typename T>
	inline std::basic_ostream<Char>& operator<<(std::basic_ostream<Char>& os, const node_view<T>& nv)
	{
		if (nv.node_)
		{
			nv.node_->visit([&os](const auto& n) { os << n; });
		}
		return os;
	}

#if !defined(DOXYGEN) && !TOML_HEADER_ONLY

	extern template class TOML_API node_view<node>;
	extern template class TOML_API node_view<const node>;

	extern template TOML_API
	std::ostream& operator<<(std::ostream&, const node_view<node>&);
	extern template TOML_API
	std::ostream& operator<<(std::ostream&, const node_view<const node>&);

	#define TOML_EXTERN(name, T)                                                                                       \
		extern template TOML_API                                                                                       \
		optional<T> node_view<node>::name<T>() const noexcept;                                                         \
		extern template TOML_API                                                                                       \
		optional<T> node_view<const node>::name<T>() const noexcept
	TOML_EXTERN(value_exact, std::string_view);
	TOML_EXTERN(value_exact, std::string);
	TOML_EXTERN(value_exact, const char*);
	TOML_EXTERN(value_exact, int64_t);
	TOML_EXTERN(value_exact, double);
	TOML_EXTERN(value_exact, date);
	TOML_EXTERN(value_exact, time);
	TOML_EXTERN(value_exact, date_time);
	TOML_EXTERN(value_exact, bool);
	TOML_EXTERN(value, std::string_view);
	TOML_EXTERN(value, std::string);
	TOML_EXTERN(value, const char*);
	TOML_EXTERN(value, signed char);
	TOML_EXTERN(value, signed short);
	TOML_EXTERN(value, signed int);
	TOML_EXTERN(value, signed long);
	TOML_EXTERN(value, signed long long);
	TOML_EXTERN(value, unsigned char);
	TOML_EXTERN(value, unsigned short);
	TOML_EXTERN(value, unsigned int);
	TOML_EXTERN(value, unsigned long);
	TOML_EXTERN(value, unsigned long long);
	TOML_EXTERN(value, double);
	TOML_EXTERN(value, float);
	TOML_EXTERN(value, date);
	TOML_EXTERN(value, time);
	TOML_EXTERN(value, date_time);
	TOML_EXTERN(value, bool);

	#if TOML_HAS_CHAR8
	TOML_EXTERN(value_exact, std::u8string_view);
	TOML_EXTERN(value_exact, std::u8string);
	TOML_EXTERN(value_exact, const char8_t*);
	TOML_EXTERN(value, std::u8string_view);
	TOML_EXTERN(value, std::u8string);
	TOML_EXTERN(value, const char8_t*);
	#endif

	#if TOML_WINDOWS_COMPAT
	TOML_EXTERN(value_exact, std::wstring);
	TOML_EXTERN(value, std::wstring);
	#endif

	#undef TOML_EXTERN

#endif // !TOML_HEADER_ONLY
}
TOML_NAMESPACE_END;

TOML_POP_WARNINGS; // TOML_DISABLE_ARITHMETIC_WARNINGS
