#pragma once

#include <cmath>
#include <memory>
#include <type_traits>

#include <boost/geometry.hpp>
#include <boost/math/differentiation/autodiff.hpp>
#include <boost/numeric/ublas/vector.hpp>

template<class T>
concept Number = std::is_integral_v<T> || std::is_floating_point_v<T>;

template<Number T>
using point = boost::geometry::model::d3::point_xyz<T>;

template<Number T>
using vector = boost::numeric::ublas::vector<T>;

template<Number T>
auto make_vector(T x, T y, T z) -> vector<T>
{
  vector<T> res(3);

  res(0) = x;
  res(1) = y;
  res(2) = z;

  return res;
}

template<Number T>
struct curve : std::enable_shared_from_this<curve<T>>
{
  enum type
  {
    circle,
    ellipse,
    helix
  };

  curve() = default;
  curve(const curve&) = default;
  auto operator=(const curve&) -> curve& = default;
  curve(curve&&) noexcept = default;
  auto operator=(curve&&) noexcept -> curve& = default;
  virtual ~curve() = default;

  virtual point<T> get_point(T t) const noexcept = 0;
  virtual vector<T> get_derivative(T t) const noexcept = 0;
  virtual T get_radii_sum() const noexcept = 0;
  virtual type get_type() const noexcept = 0;
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

  point<T> get_point(T t) const noexcept override
  {
    // x(t) = r cos(t)
    // y(t) = r sin(t)

    return point<T>(m_radius * std::cos(t), m_radius * std::sin(t), 0);
  }

  vector<T> get_derivative(T t) const noexcept override
  {
    using namespace boost::math::differentiation;

    constexpr unsigned Order = 5;
    auto x = cos(make_fvar<T, Order>(t));
    x *= m_radius;

    auto y = sin(make_fvar<T, Order>(t));
    y *= m_radius;

    return make_vector<T>(x.derivative(1), y.derivative(1), 0);
  }

  curve<T>::type get_type() const noexcept override
  {
    return curve<T>::type::circle;
  }

  T get_radius() const noexcept { return m_radius; }

  T get_radii_sum() const noexcept override { return m_radius; }
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

  point<T> get_point(T t) const noexcept override
  {
    // x(t) = a cos(t)
    // y(t) = b sin(t)

    return point<T>(m_x_radii * std::cos(t), m_y_radii * std::sin(t), 0);
  }

  vector<T> get_derivative(T t) const noexcept override
  {
    using namespace boost::math::differentiation;

    constexpr unsigned Order = 5;
    auto x = cos(make_fvar<T, Order>(t));
    x *= m_x_radii;

    auto y = sin(make_fvar<T, Order>(t));
    y *= m_y_radii;

    return make_vector<T>(x.derivative(1), y.derivative(1), 0);
  }

  curve<T>::type get_type() const noexcept override
  {
    return curve<T>::type::ellipse;
  }

  T get_x_radii() const noexcept { return m_x_radii; }

  T get_y_radii() const noexcept { return m_y_radii; }

  T get_radii_sum() const noexcept override { return m_x_radii + m_y_radii; }
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

  point<T> get_point(T t) const noexcept override
  {
    // x(t) = a cos(t)
    // y(t) = a sin(t)
    // z(t) = b t

    return point<T>(m_radius * std::cos(t), m_radius * std::sin(t), m_step * t);
  }

  vector<T> get_derivative(T t) const noexcept override
  {
    using namespace boost::math::differentiation;

    constexpr unsigned Order = 5;
    auto x = cos(make_fvar<T, Order>(t));
    x *= m_radius;

    auto y = sin(make_fvar<T, Order>(t));
    y *= m_radius;

    auto z = make_fvar<T, Order>(t);
    z *= m_step;

    return make_vector<T>(x.derivative(1), y.derivative(1), z.derivative(1));
  }

  curve<T>::type get_type() const noexcept override
  {
    return curve<T>::type::helix;
  }

  T get_radius() const noexcept { return m_radius; }

  T get_step() const noexcept { return m_step; }

  T get_radii_sum() const noexcept override { return m_radius; }
};
