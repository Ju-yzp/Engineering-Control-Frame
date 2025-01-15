#ifndef REGISTER_PTR_MACROS_HPP_
#define REGISTER_PTR_MACROS_HPP_

#define REGISTER_SAMRT_PTR(...)\
        SHARED_PTR_DEFINITIONS(__VA_ARGS__)\
        WEAK_PTR_DEFINITIONS(__VA_ARGS__)\
        UNIQUE_PTR_DEFINITIONS(__VA_ARGS__)

#define SHARED_PTR_DEFINITIONS(__VA_ARGS__)\
        using ConstSharedPtr = std::shared_ptr<const __VA_ARGS__>;\
        using SharedPtr = std::shared_ptr<__VA_ARGS__>;

#define WEAK_PTR_DEFINITIONS(__VA_ARGS__)\
        using ConstWeakPtr = std::weak_ptr<const __VA_ARGS__>;\
        using WeakPtr = std::weak_ptr<__VA_ARGS__>;

#define UNIQUE_PTR_DEFINITIONS(__VA_ARGS__)\
        using ConstUniquePtr = std::unique_ptr<const __VA_ARGS__>;\
        using UniquePtr = std::unique_ptr<__VA_ARGS__>;

#endif