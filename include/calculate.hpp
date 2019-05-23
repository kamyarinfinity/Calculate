/*
    Calculate - Version 2.1.1rc10
    Last modified 2018/07/28
    Released under MIT license
    Copyright (c) 2016-2018 Alberto Lorenzo <alorenzo.md@gmail.com>
*/


#ifndef __CALCULATE_HPP__
#define __CALCULATE_HPP__

#undef strtoll
#include <nlohmann/json.hpp>
#include "calculate/parser.hpp"
#include <map>

namespace calculate {

namespace defaults {

template<typename T> T add(const T& x, const T& y) noexcept { return x + y; }
template<typename T> T sub(const T& x, const T& y) noexcept { return x - y; }
template<typename T> T mul(const T& x, const T& y) noexcept { return x * y; }
template<typename T> T div(const T& x, const T& y) noexcept { return x / y; }

struct Precedence {
    static constexpr std::size_t very_low = 1111u;
    static constexpr std::size_t low = 2222u;
    static constexpr std::size_t normal = 5555u;
    static constexpr std::size_t high = 8888u;
    static constexpr std::size_t very_high = 9999u;
};

}


using nlohmann::json;

template <> BaseParser<double>::BaseParser(const Lexer& lexer) :
	_lexer{ lexer.clone() },
	constants{ _lexer.get() },
	functions{ _lexer.get() },
	operators{ _lexer.get() },
	prefixes{ _lexer.get() },
	suffixes{ _lexer.get() },
	optimize{ false }
{
	using namespace defaults;
	using F1 = Type(*)(Type);
	using F2 = Type(*)(Type, Type);
	using F3 = Type(*)(Type, Type, Type);

	auto pow = [](Type x1, Type x2) noexcept {
		if (x2 <= 0. || x2 > 256 || std::trunc(x2) != x2)
			return std::pow(x1, x2);

		auto exp = static_cast<int>(x2);
		auto prod = 1.;
		while (exp) {
			if (exp & 1)
				prod *= x1;
			exp >>= 1;
			x1 *= x1;
		}
		return prod;
	};

	auto fact = [](Type x) noexcept {
		if (x > 256)
			return std::numeric_limits<Type>::infinity();

		auto prod = 1.;
		for (auto i = 2.; i <= x; i++)
			prod *= i;
		return prod;
	};

	constants.insert({
		{"pi", 3.14159265358979323846},
		{"e", 2.71828182845904523536},
		{"phi", 1.61803398874989484820},
		{"gamma", 0.57721566490153286060}
		});

	functions.insert({
		{"id", [](Type x) noexcept { return x; }},
		{"neg", [](Type x) noexcept { return -x; }},
		{"inv", [](Type x) noexcept { return Type{1} / x; }},
		{"fabs", static_cast<F1>(std::fabs)},
		{"abs", static_cast<F1>(std::abs)},
		{"fma", static_cast<F3>(std::fma)},
		{"copysign", static_cast<F2>(std::copysign)},
		{"nextafter", static_cast<F2>(std::nextafter)},
		{"fdim", static_cast<F2>(std::fdim)},
		{"fmax", static_cast<F2>(std::fmax)},
		{"fmin", static_cast<F2>(std::fmin)},
		{"fdim", static_cast<F2>(std::fdim)},
		{"max", static_cast<F2>(std::fmax)},
		{"min", static_cast<F2>(std::fmin)},
		{"ceil", static_cast<F1>(std::ceil)},
		{"floor", static_cast<F1>(std::floor)},
		{"fmod", static_cast<F2>(std::fmod)},
		{"trunc", static_cast<F1>(std::trunc)},
		{"round", static_cast<F1>(std::round)},
		{"rint", static_cast<F1>(std::rint)},
		{"nearbyint", static_cast<F1>(std::nearbyint)},
		{"remainder", static_cast<F2>(std::remainder)},
		{"pow", static_cast<F2>(std::pow)},
		{"sqrt", static_cast<F1>(std::sqrt)},
		{"cbrt", static_cast<F1>(std::cbrt)},
		{"hypot", static_cast<F2>(std::hypot)},
		{"exp", static_cast<F1>(std::exp)},
		{"expm1", static_cast<F1>(std::expm1)},
		{"exp2", static_cast<F1>(std::exp2)},
		{"log", static_cast<F1>(std::log)},
		{"log10", static_cast<F1>(std::log10)},
		{"log1p", static_cast<F1>(std::log1p)},
		{"log2", static_cast<F1>(std::log2)},
		{"logb", static_cast<F1>(std::logb)},
		{"sin", static_cast<F1>(std::sin)},
		{"cos", static_cast<F1>(std::cos)},
		{"tan", static_cast<F1>(std::tan)},
		{"asin", static_cast<F1>(std::asin)},
		{"acos", static_cast<F1>(std::acos)},
		{"atan", static_cast<F1>(std::atan)},
		{"atan2", static_cast<F2>(std::atan2)},
		{"sinh", static_cast<F1>(std::sinh)},
		{"cosh", static_cast<F1>(std::cosh)},
		{"tanh", static_cast<F1>(std::tanh)},
		{"asinh", static_cast<F1>(std::asinh)},
		{"acosh", static_cast<F1>(std::acosh)},
		{"atanh", static_cast<F1>(std::atanh)},
		{"erf", static_cast<F1>(std::erf)},
		{"erfc", static_cast<F1>(std::erfc)},
		{"tgamma", static_cast<F1>(std::tgamma)},
		{"lgamma", static_cast<F1>(std::lgamma)},
		{"fact", fact}
		});

	operators.insert({
		{"+", {add<Type>, Precedence::low, Associativity::FULL}},
		{"-", {sub<Type>, Precedence::low, Associativity::LEFT}},
		{"*", {mul<Type>, Precedence::normal, Associativity::FULL}},
		{"/", {div<Type>, Precedence::normal, Associativity::LEFT}},
		{"%", {static_cast<F2>(std::fmod), Precedence::normal, Associativity::LEFT}},
		{"^", {pow, Precedence::high, Associativity::RIGHT}}
		});

	prefixes.insert({
		{"+", "id"},
		{"-", "neg"}
		});

	suffixes.insert({
		{"!", "fact"}
		});
}

template <> BaseParser<int>::BaseParser(const Lexer& lexer) :
	_lexer{ lexer.clone() },
	constants{ _lexer.get() },
	functions{ _lexer.get() },
	operators{ _lexer.get() },
	prefixes{ _lexer.get() },
	suffixes{ _lexer.get() },
	optimize{ false }
{
	using namespace defaults;
	using F1 = Type(*)(Type);
	using F2 = Type(*)(Type, Type);
	using F3 = Type(*)(Type, Type, Type);

	auto pow = [](Type x1, Type x2) noexcept -> Type {
		if (x2 <= 0 || x2 > 256 || std::trunc(x2) != x2)
			return std::pow(x1, x2);

		Type exp = x2;
		Type prod = 1;
		while (exp) {
			if (exp & 1)
				prod *= x1;
			exp >>= 1;
			x1 *= x1;
		}
		return prod;
	};

	auto fact = [](Type x) noexcept {
		if (x > 256)
			return std::numeric_limits<Type>::infinity();

		auto prod = 1;
		for (auto i = 2; i <= x; i++)
			prod *= i;
		return prod;
	};

	auto mini = [](Type x1, Type x2) noexcept {
		return x1 > x2 ? x2 : x1;
	};
	auto maxi = [](Type x1, Type x2) noexcept {
		return x1 > x2 ? x1 : x2;
	};
	auto mod = [](Type x1, Type x2) noexcept {
		return x1 % x2;
	};

	functions.insert({
		{"id", [](Type x) noexcept { return x; }},
		{"neg", [](Type x) noexcept { return -x; }},
		{"abs", static_cast<F1>(std::abs)},
		{"max", static_cast<F2>(maxi)},
		{"min", static_cast<F2>(mini)},
		{"fact", fact}
		});

	operators.insert({
		{"+", {add<Type>, Precedence::low, Associativity::FULL}},
		{"-", {sub<Type>, Precedence::low, Associativity::LEFT}},
		{"*", {mul<Type>, Precedence::normal, Associativity::FULL}},
		{"/", {div<Type>, Precedence::normal, Associativity::LEFT}},
		{"%", {static_cast<F2>(mod), Precedence::normal, Associativity::LEFT}},
		{"^", {static_cast<F2>(pow), Precedence::high, Associativity::RIGHT}}
		});

	prefixes.insert({
		{"+", "id"},
		{"-", "neg"}
		});

	suffixes.insert({
		{"!", "fact"}
		});
}

template <typename Type>
struct Result {
	std::string variable;
	Type value;
	template <typename Type2>
	static Result<Type> from(const Result<Type2> &r2) {
		Result<Type> r1;
		r1.variable = r2.variable;
		r1.value = (Type)r2.value;
		return r1;
	}
};

template <typename Type>
class GenericExpression {
	using Parser = BaseParser<Type>;
	using MathExp = typename Parser::Expression;
	Parser parser;
	std::shared_ptr<MathExp> expression;
	std::string assign_var;
	char assign_type;
public:
	GenericExpression(const GenericExpression& ge):parser(ge.parser) {
		expression = ge.expression;
		assign_var = ge.assign_var;
		assign_type = ge.assign_type;
	}
	GenericExpression(std::string expr) :parser(make_lexer<Type>()),assign_var(""),assign_type(0) {
		size_t len = expr.length();
		if (len == 0) return;
		size_t idx = expr.find("=");
		size_t expr_start = 0;
		if (idx != std::string::npos)
		{
			if (idx == 0 || idx == len-1) throw std::runtime_error("Malformed expression");

			if (expr[idx + 1] == '=')
			{
				// the first equal token is part of equality check -> no assignment
				expression = std::make_shared<MathExp>(parser.parse(expr));
				return;
			}

			size_t last_idx = idx;
			if (expr[idx - 1] == '+' ||
				expr[idx - 1] == '-' ||
				expr[idx - 1] == '/' ||
				expr[idx - 1] == '%' ||
				expr[idx - 1] == '*' ||
				expr[idx - 1] == '^')
			{
				last_idx = idx - 1;
				assign_type = expr[idx - 1];
			}
			else assign_type = '=';

			// trim
			assign_var = expr.substr(0, last_idx);
			const std::string whitespaces = " \n\r\t\v\f";
			size_t first = assign_var.find_first_not_of(whitespaces);
			size_t last = assign_var.find_last_not_of(whitespaces);
			assign_var = assign_var.substr(first, last - first + 1);
			if (assign_var.length() == 0) throw std::runtime_error("Malformed expression");

			expr_start = idx + 1;
		}
		expression = std::make_shared<MathExp>(parser.parse(expr.substr(expr_start)));
	}
	std::vector<std::string> variables() {
		return expression->variables();
	}
	std::string assign_variable() {
		return assign_var;
	}
	Result<Type> eval(std::map<std::string,Type> args) {
		Result<Type> r;
		auto vars = variables();
		auto var_vals = std::vector<Type>(vars.size());
		int i = 0;
		for (auto v : vars) {
			var_vals[i++] = args[v];
		}
		Type val = (*expression)(var_vals);
		switch (assign_type) {
		case '+':
			r.value = args[assign_var] + val;
			break;
		case '-':
			r.value = args[assign_var] - val;
			break;
		case '*':
			r.value = args[assign_var] * val;
			break;
		case '/':
			r.value = args[assign_var] / val;
			break;
		case '^':
			r.value = std::pow(args[assign_var], val);
			break;
		case '%':
			r.value = mod(args[assign_var], val);
			break;
		case '=':
		default:
			r.value = val;
			break;
		}
		r.variable = assign_var;
		return r;
	}
private:
	double mod(double a, double b) { return std::fmod(a, b); }
	int mod(int a, int b) { return a % b; }
};

class Expression {
private:
	std::unique_ptr<GenericExpression<double>> double_expression;
	std::unique_ptr<GenericExpression<int>> int_expression;
public:
	enum Type {
		DOUBLE,
		INTEGER
	} type;
	Expression(Expression &&exp) noexcept {
		if (exp.type == INTEGER)
		{
			type = INTEGER;
			int_expression = std::move(exp.int_expression);
		}
		else
		{
			type = DOUBLE;
			double_expression = std::move(exp.double_expression);
		}
	}
	Expression(const Expression &exp) {
		if (exp.type == INTEGER)
		{
			type = INTEGER;
			int_expression = std::make_unique<GenericExpression<int>>(*exp.int_expression);
		}
		else
		{
			type = DOUBLE;
			double_expression = std::make_unique<GenericExpression<double>>(*exp.double_expression);
		}
	}
	Expression(std::string expression_str, json context) {
		double_expression = std::make_unique<GenericExpression<double>>(expression_str);
		bool needs_integer_parser = true;
		for (const std::string &var_str : double_expression->variables()) {
			// default any variable outside event-system state to double type, for now
			needs_integer_parser = needs_integer_parser && (context.find(var_str) != context.end() &&
				context[var_str].is_number_integer());
		}
		if (needs_integer_parser) {
			double_expression.reset();
			int_expression = std::make_unique<GenericExpression<int>>(expression_str);
			type = INTEGER;
		}
		else {
			type = DOUBLE;
		}
	}
	Type get_type() { return type; }
	std::string get_assign_variable() {
		if (type == DOUBLE)
			return double_expression->assign_variable();
		else if(type == INTEGER)
			return int_expression->assign_variable();
		throw std::runtime_error("Unsupported expression type");
	}
	std::vector<std::string> get_variables() {
		if (type == DOUBLE)
			return double_expression->variables();
		else if (type == INTEGER)
			return int_expression->variables();
		throw std::runtime_error("Unsupported expression type");
	}
	template <typename T>
	Result<T> eval(json context) {
		throw std::runtime_error("Unsupported eval type for expression");
	}
	template <>
	Result<int> eval(json context) {
		if (type == INTEGER)
			return int_expression->eval(context);
		throw std::runtime_error("Cannot eval: expression type is not integer.");
	}
	template <>
	Result<double> eval(json context) {
		if (type == DOUBLE)
			return double_expression->eval(context);
		throw std::runtime_error("Cannot eval: expression type is not integer.");
	}
	// these eval versions will cast if type is different
	Result<double> eval_to_double(json context) {
		if (type == DOUBLE) return double_expression->eval(context);
		else if (type == INTEGER) return Result<double>::from(int_expression->eval(context));
		throw std::runtime_error("Unknown expression type");
	}
	Result<int> eval_to_int(json context) {
		if (type == DOUBLE) return Result<int>::from(double_expression->eval(context));
		else if (type == INTEGER) return int_expression->eval(context);
		throw std::runtime_error("Unknown expression type");
	}
};

}

#endif
