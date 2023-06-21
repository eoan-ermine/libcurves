#pragma once

#include <cmath>
#include <memory>
#include <type_traits>

#include <boost/geometry.hpp>
#include <boost/math/differentiation/autodiff.hpp>
#include <boost/numeric/ublas/vector.hpp>

template<class T>
concept Number = std::is_integral_v<T> || std::is_floating_point_v<T>;

template<typename T>
using point = boost::geometry::model::d3::point_xyz<T>;

template<typename T>
using vector = boost::numeric::ublas::scalar_vector<T>;

template<Number T>
struct curve : std::enable_shared_from_this<curve<T>>
{
  enum type {
    circle, ellipse, helix
  };

  curve(const curve&) = default;
  auto operator=(const curve&) -> curve& = default;
  curve(curve&&) noexcept = default;
  auto operator=(curve&&) noexcept -> curve& = default;
  virtual ~curve() = default;

  virtual auto get_point(T t) -> point<T> = 0;
  virtual auto get_derivative(T t) -> vector<T> = 0;
  virtual auto get_type() const noexcept -> type = 0;
};

template<Number T>
class circle : public curve<T>
{
  T m_radius;

public:
  explicit circle(T radius)
      : m_radius(radius)
  {
  }

  auto get_point(T t) const noexcept -> point<T>
  {
    // x(t) = r cos(t)
    // y(t) = r sin(t)

    return point<T>(m_radius * std::cos(t), m_radius * std::sin(t));
  }

  auto get_derivative(T t) const noexcept -> vector<T>
  {
    using namespace boost::math::differentiation;

    constexpr unsigned Order = 5;
    auto x = cos(make_fvar<T, Order>(t));
    x *= m_radius;

    auto y = sin(make_fvar<T, Order>(t));
    y *= m_radius;

    return vector<T>(x.derivative(1), y.derivative(1), 0);
  }

  auto get_type() const noexcept -> curve<T>::type {
    return curve<T>::type::circle;
  }
};

template<Number T>
class ellipse : public curve<T>
{
  T m_x_radii, m_y_radii;

public:
  ellipse(T x_radii, T y_radii)
      : m_x_radii(x_radii)
      , m_y_radii(y_radii)
  {
  }

  auto get_point(T t) const noexcept -> point<T>
  {
    // x(t) = a cos(t)
    // y(t) = b sin(t)

    return point<T>(m_x_radii * std::cos(t), m_y_radii * std::sin(t));
  }

  auto get_derivative(T t) const noexcept -> vector<T>
  {
    using namespace boost::math::differentiation;

    constexpr unsigned Order = 5;
    auto x = cos(make_fvar<T, Order>(t));
    x *= m_x_radii;

    auto y = sin(make_fvar<T, Order>(t));
    y *= m_y_radii;

    return vector<T>(x.derivative(1), y.derivative(1), 0);
  }

  auto get_type() const noexcept -> curve<T>::type {
    return curve<T>::type::ellipse;
  }
};

template<Number T>
class helix : public curve<T>
{
  T m_radius, m_step;

public:
  helix(T radius, T step)
      : m_radius(radius)
      , m_step(step)
  {
  }

  auto get_point(T t) const noexcept -> point<T>
  {
    // x(t) = a cos(t)
    // y(t) = a sin(t)
    // z(t) = b t

    return point<T>(m_radius * std::cos(t), m_radius * std::sin(t), m_step * t);
  }

  auto get_derivative(T t) const noexcept -> vector<T>
  {
    using namespace boost::math::differentiation;

    constexpr unsigned Order = 5;
    auto x = cos(make_fvar<T, Order>(t));
    x *= m_radius;

    auto y = sin(make_fvar<T, Order>(t));
    y *= m_radius;

    auto z = make_fvar<T, Order>(t);
    z *= m_step;

    return vector<T>(x.derivative(1), y.derivative(1), z.derivative(1));
  }

  auto get_type() const noexcept -> curve<T>::type {
    return curve<T>::type::helix;
  }
};
