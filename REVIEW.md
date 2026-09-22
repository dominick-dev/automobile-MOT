# Review Instructions
This is a solo portfolio project. This file keeps
automated review focused on real issues and code quality, not
formatting nits.

## Skip
- Formatting/naming — clang-format and clang-tidy already handle this.
- Vague suggestions with no specific line or concrete reasoning
  ("consider adding more logging/comments" with nothing specific).

## Focus on

**Correctness**
- Logic errors and edge cases
- Sign, unit, and coordinate-frame bugs (radar/geometry-heavy codebase so
  position, velocity, and angle conventions matter)
- Memory safety (dangling references, iterator invalidation after
  vector reallocation, use-after-move)
- Off-by-one and boundary bugs
- Missing or wrong error handling
- Test coverage gaps for new or changed logic

**Modern, efficient C++ 
- Raw `new`/`delete` where RAII (`unique_ptr`, containers) should be used
- Unnecessary copies — pass-by-value where a `const&` would do, missing
  `std::move` when a parameter is stored into a struct/container
- Raw pointers used for ownership or where a reference would express
  intent more clearly
- Missing `const` on values/params/methods that aren't mutated
- C-style casts instead of `static_cast`/`static_pointer_cast`/etc.
- `#define` or function-macros where `constexpr`/inline functions/templates
  would work
- Raw arrays/manual indexing where `std::array`/`vector` + algorithms
  (`std::sort`, `find_if`, `transform`, `erase_if`, etc.) fit better
- Unnecessary heap allocation in a hot path (per-frame processing code
  especially — allocations inside the per-detection or per-cluster loop
  are worth flagging even if not strictly a bug)
- Manual loops that a standard algorithm or ranges view expresses more
  clearly, when it wouldn't hurt readability

## Tone
Be direct and specific, point to the exact line and either the concrete
failure mode (for correctness) or the concrete idiom to use instead
(for style/modernity). If something is genuinely a matter of preference
with no real efficiency or clarity cost, don't comment at all.

Also end your pr review comment with an emoji
