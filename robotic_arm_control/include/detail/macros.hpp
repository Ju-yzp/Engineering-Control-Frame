#ifndef REGISTER_PTR_MACROS_HPP_
#define REGISTER_PTR_MACROS_HPP_

#include<memory>

#define REGISTER_SAMRT_PTR(...)\
        SHARED_PTR_DEFINITIONS(_VA_ARGS_)\
        WEAK_PTR_DEFINITIONS(_VA_ARGS_)\
        UNIQUE_PTR_DEFINITIONS(_VA_ARGS_)

#define SHARED_PTR_DEFINITIONS(_VA_ARGS_)\
        using ConstSharedPtr = std::shared_ptr<const _VA_ARGS_>;\
        using SharedPtr = std::shared_ptr<_VA_ARGS_>;

#define WEAK_PTR_DEFINITIONS(_VA_ARGS_)\
        using ConstWeakPtr = std::weak_ptr<const _VA_ARGS_>;\
        using WeakPtr = std::weak_ptr<_VA_ARGS_>;

#define UNIQUE_PTR_DEFINITIONS(_VA_ARGS_)\
        using ConstUniquePtr = std::unique_ptr<const _VA_ARGS_>;\
        using UniquePtr = std::unique_ptr<_VA_ARGS_>;

#endif