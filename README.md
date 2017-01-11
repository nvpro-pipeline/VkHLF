# Vulkan High Level Framework

VkHLF is an experimental high level abstraction library on top of Vulkan. It adds features like transparent suballocation,
resource tracking on the CPU & GPU and simplified resource creation while staying as close as possible to the original
Vulkan API. In contrast to Vulkan-Hpp, which was carefully designed to be a zero-overhead C++ abstraction for Vulkan, this 
library adds significant higher-level functionality. Even so, it has been designed for high-performance, but it can cost
performance relative to native Vulkan if not employed with the intended usage patterns. 

Since this project is in its early stages and under heavy development expect bugs and interface changes for a while. It should
not be used for production code yet!

# Build instructions

* Clone the repository.
* Execute git submodule update --init --recursive to get the 3rdparty dependencies. Alternatevely unpack glfw3 in 3rdparty/glfw.
* Install Vulkan SDK 1.0.37 or newer.
* Run CMake to generate the makefiles of your choice.

# VkHLF namespace
All handles and structs which hold handles have been replicated in the ```vkhlf``` namespace. For the other structs we simply
use Vulkan-Hpp C++ bindings.

# VkHLF Reference Counting
Vulkan requires the developer to keep the object hierarchy intact all the time. That is, destroying an object
like the ```vk::Device``` before any of the objects created from it results in undefined behavior. It is also useful for a
```vk::CommandPool``` not to be destroyed if there are any ```vk::CommandBuffer``` objects left created by the same pool.
To ensure that functionality VkHLF is using ```std::shared_ptr``` and ```std::weak_ptr``` to keep track of the parents or children
of objects, i.e. a ```vk::DeviceMemory``` object is a child object of a ```vk::Device``` object, so it has to keep a ```std::shared_ptr``` to
the ```vk::Device``` used for creation. This way the ```vk::Device``` stays alive as long as any ```vk::DeviceMemory``` or
other child object exists and the destructor of any object has all the information required to destroy an object.

As a consequence to that structs containing handles or arrays of handles are no longer binary compatible with the native
Vulkan API. This means that each time such a struct or array is being passed to Vulkan VkHLF has to copy data into
a temporary struct or array and pass this one to Vulkan. Luckily this is not as bad as it seems since most of those
copy operations are only at initialization time, but not during ```vk::CommandBuffer``` construction.

# VkHLF Device Suballocator
Vulkan moves the device suballocation responsibility from the driver to the application. While it is usually possible
to do thousands of buffer allocations in OpenGL without any failure it is likely to get a failure on Vulkan when
doing one device allocation for each VkBuffer due to OS limitations. Thus application developers now have to implement
device suballocators and keep track of the device/offset pair in the application. To simplify application development
VkHLF provides the vkhlf::DeviceSuballocator interface which can be used for suballocation. The suballocator returns
```vkhlf::DeviceMemory``` objects which always store the ```offset``` in addition to the ```vk::DeviceMemory``` object.
Each location which is using ```vkhlf::DeviceMemory``` will use the pair of ```vk::DeviceMemory``` and ```offset``` 
completely transparent to the developer.

# VkHLF GPU Resource Tracking
Vulkan does not have any automatic resource tracking. It is possible to change or delete resources which are currently
in use on the GPU which will most likely result in strage behaviour, or worst case in an application or system crash.

It is essential to ensure that resources do not get destroyed while they are still in use by the GPU. Thus we have implemented
a ```vkhlf::ResourceTracker``` interface used by ```vkhlf::CommandBuffer``` to track all resources used to build up the 
command buffer.

In its current state resource tracking is an expensive operation if it is done on the fly while building up command buffers. 
Developers should try to avoid this cost by using one of the following techniques:
* If possible, reuse command buffers over multiple frames.
* Otherwise, reuse the tracking data. If it is known that the set of resources used by a command buffer does not change over multiple
frames one can implement a version of the resource tracking interface which takes the tracking information of another resource tracker
to keep the resources alive. In release mode this tracker does nothing, in debug mode it should validate that all used resources are
already tracked.


# Providing Pull Requests
NVIDIA is happy to review and consider pull requests for merging into the main tree of the nvpro-pipeline for bug fixes and features. Before providing a pull request to NVIDIA, please note the following:

* A pull request provided to this repo by a developer constitutes permission from the developer for NVIDIA to merge the provided changes or any NVIDIA modified version of these changes to the repo. NVIDIA may remove or change the code at any time and in any way deemed appropriate.
* Not all pull requests can be or will be accepted. NVIDIA will close pull requests that it does not intend to merge.
* The modified files and any new files must include the unmodified NVIDIA copyright header seen at the top of all shipping files.
