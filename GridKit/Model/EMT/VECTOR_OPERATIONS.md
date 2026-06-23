# Vectorized Equation Support For PhasorDynamics Components

## Summary

Add a small header-only equation notation layer under PhasorDynamics so component residual kernels can write N-phase vector equations like:

```cpp
r1 = i1 - Y_ * v;
r2 = i2 - Y_ * v;
```

The feature must preserve the existing PhasorDynamics/Enzyme architecture: residuals remain `ScalarT*` pointer kernels, state/residual storage remains flat and contiguous, and sparse Jacobians continue to use the existing `DfDy`, `DfDwb`, `DhDy`, and `DhDwb` helpers.

## Key Changes

Add [VectorizedEquations.hpp](/home/lukel/GridKit/GridKit/Model/PhasorDynamics/VectorizedEquations.hpp) and install it through `phasor_dynamics_core` in [CMakeLists.txt](/home/lukel/GridKit/GridKit/Model/PhasorDynamics/CMakeLists.txt:8).

Public API:

```cpp
namespace GridKit::PhasorDynamics::Equation
{
  template <class T, size_t N> struct Vec;
  template <class T, size_t N> struct VecRef;
  template <class T, size_t R, size_t C> struct Mat;

  template <size_t N, class T>
  VecRef<T, N> slice(T* base, size_t offset);

  template <size_t N, class T>
  VecRef<T, N> block(T* base, size_t block_index);
}
```

Core behavior:

```cpp
template <class T, size_t N>
struct Vec
{
  using value_type = T;
  static constexpr size_t size = N;
  std::array<T, N> data{};

  __attribute__((always_inline)) T& operator[](size_t i) { return data[i]; }
  __attribute__((always_inline)) const T& operator[](size_t i) const { return data[i]; }
};

template <class T, size_t N>
struct VecRef
{
  using value_type = T;
  static constexpr size_t size = N;
  T* data{nullptr};

  __attribute__((always_inline)) T& operator[](size_t i) { return data[i]; }
  __attribute__((always_inline)) const T& operator[](size_t i) const { return data[i]; }

  template <class Expr>
  __attribute__((always_inline)) VecRef& operator=(const Expr& rhs)
  {
    static_assert(Expr::size == N);
    for (size_t i = 0; i < N; ++i)
      data[i] = rhs[i];
    return *this;
  }
};

template <class T, size_t R, size_t C>
struct Mat
{
  using value_type = T;
  static constexpr size_t rows = R;
  static constexpr size_t cols = C;
  std::array<T, R * C> data{};

  __attribute__((always_inline)) T& operator()(size_t r, size_t c) { return data[r * C + c]; }
  __attribute__((always_inline)) const T& operator()(size_t r, size_t c) const { return data[r * C + c]; }
};
```

Required operators for v1:

```cpp
// Vector addition/subtraction for any Vec/VecRef combination.
template <class L, class R>
__attribute__((always_inline)) auto operator-(const L& lhs, const R& rhs);

template <class L, class R>
__attribute__((always_inline)) auto operator+(const L& lhs, const R& rhs);

// Matrix-vector multiply for Mat * Vec or Mat * VecRef.
template <class T, size_t R, size_t C, class V>
__attribute__((always_inline)) auto operator*(const Mat<T, R, C>& A, const V& x);
```

Implement operators as fixed-size loops returning `Vec<OutT, N>`, where `OutT` is derived with `decltype(lhs[0] - rhs[0])` or `decltype(A(0, 0) * x[0])`. Do not use heap allocation, `std::vector`, Eigen, BLAS, or runtime-sized expression objects inside residual kernels.

## Target Usage

For two vectorized equations:

```math
0 = i_1 - Yv
0 = i_2 - Yv
```

use flat storage with named vector blocks:

```cpp
static constexpr size_t N = 3;

static constexpr size_t I1 = 0;
static constexpr size_t I2 = 1;

static constexpr size_t V  = 0;

static constexpr size_t R1 = 0;
static constexpr size_t R2 = 1;
```

Residual implementation:

```cpp
template <typename scalar_type, typename index_type>
__attribute__((always_inline)) int MyModel<scalar_type, index_type>::evaluateInternalResidual(
    ScalarT* y, [[maybe_unused]] ScalarT* yp, ScalarT* wb, ScalarT* f)
{
  namespace Eq = GridKit::PhasorDynamics::Equation;

  auto i1 = Eq::block<N>(y, I1);
  auto i2 = Eq::block<N>(y, I2);
  auto v  = Eq::block<N>(wb, V);

  auto r1 = Eq::block<N>(f, R1);
  auto r2 = Eq::block<N>(f, R2);

  r1 = i1 - Y_ * v;
  r2 = i2 - Y_ * v;

  return 0;
}
```

Equivalent flat layout:

```cpp
y  = [i1_a, i1_b, i1_c, i2_a, i2_b, i2_c]
wb = [v_a,  v_b,  v_c]
f  = [r1_a, r1_b, r1_c, r2_a, r2_b, r2_c]
```

For mixed scalar/vector layouts, use `slice<N>(ptr, exact_offset)` instead of `block<N>(ptr, block_index)`.

## Enzyme Integration

No changes to `GridKit/AutomaticDifferentiation/Enzyme` are required for this feature.

Each vectorized component follows the existing PhasorDynamics pattern:

```cpp
int MyModel<scalar_type, index_type>::evaluateJacobian()
{
  updateExternalVector();

  J_.zeroMatrix();

  const size_t max_nnz =
      std::max(f_.size() * y_.size(), f_.size() * wb_.size());

  if (J_rows_buffer_ == nullptr)
  {
    J_rows_buffer_ = new IdxT[max_nnz];
    J_cols_buffer_ = new IdxT[max_nnz];
    J_vals_buffer_ = new RealT[max_nnz];
  }

  GridKit::Enzyme::Sparse::DfDy<MyModel<ScalarT, IdxT>,
                                GridKit::Enzyme::Sparse::MemberFunctions::InternalResidual,
                                ScalarT,
                                IdxT>::eval(this,
                                            f_.size(),
                                            y_.size(),
                                            residual_indices_.data(),
                                            variable_indices_.data(),
                                            y_.data(),
                                            yp_.data(),
                                            wb_.data(),
                                            J_rows_buffer_,
                                            J_cols_buffer_,
                                            J_vals_buffer_,
                                            J_);

  GridKit::Enzyme::Sparse::DfDwb<MyModel<ScalarT, IdxT>,
                                 GridKit::Enzyme::Sparse::MemberFunctions::InternalResidual,
                                 ScalarT,
                                 IdxT>::eval(this,
                                             f_.size(),
                                             wb_.size(),
                                             residual_indices_.data(),
                                             wb_indices_.data(),
                                             y_.data(),
                                             yp_.data(),
                                             wb_.data(),
                                             J_rows_buffer_,
                                             J_cols_buffer_,
                                             J_vals_buffer_,
                                             J_);

  return 0;
}
```

Each vectorized component that packs external bus variables must own:

```cpp
std::vector<ScalarT> wb_;
std::vector<IdxT>    wb_indices_;
```

and update both before residual and Jacobian evaluation:

```cpp
void updateExternalVector()
{
  for (size_t n = 0; n < N; ++n)
  {
    wb_[n]         = bus_->y()[n];
    wb_indices_[n] = bus_->getVariableIndices()[n];
  }
}
```

For differential vector equations using `yp`, call the existing `DfDy::eval(..., alpha_, ...)` overload so Enzyme contributes `alpha * df/dyp`.

## Test Plan

Add focused unit tests for the notation layer and one Enzyme-backed model test.

Test cases:

- `VecRef` block assignment writes to the expected flat `ScalarT*` offsets.
- `r = i - Y * v` matches hand-computed values for dense and diagonal `Y`.
- Two residual blocks produce the exact flat residual order `[r1, r2]`.
- Enzyme sparse Jacobian for `r1 = i1 - Yv`, `r2 = i2 - Yv` has:
  - identity block for `dr1/di1`,
  - identity block for `dr2/di2`,
  - no cross terms between `r1` and `i2` or `r2` and `i1`,
  - two `-Y` blocks for `dr1/dv` and `dr2/dv`.
- Repeat with diagonal `Y` to verify autosparsity drops structural zero off-diagonal entries.

Use the existing PhasorDynamics unit-test and Enzyme build pattern. Do not add a separate EMT system model.

## Assumptions

- Phase count is compile-time for this first implementation, with `N = 3` as the immediate target.
- The helper header lives in PhasorDynamics because the implementation will use the current PhasorDynamics component/system architecture.
- The notation layer is only a residual-kernel convenience; it does not own model state, allocate memory, or assemble Jacobians manually.
- Existing pointer-based residual wrappers remain the Enzyme boundary.
