#ifndef TEST_PLUGIN_H_INCLUDED_
#define TEST_PLUGIN_H_INCLUDED_ 1

#include <SharedMemory/plugins/b3PluginContext.h>
#include <SharedMemory/SharedMemoryPublic.h>
#include <SharedMemory/SharedMemoryCommon.h>

/* Plugin function prototypes */
#ifdef __cplusplus
extern "C"
{
#endif
	B3_SHARED_API int initPlugin(b3PluginContext* context);
	B3_SHARED_API void exitPlugin(b3PluginContext* context);
	B3_SHARED_API int executePluginCommand(b3PluginContext* context, const b3PluginArguments* arguments);
  B3_SHARED_API int processNotifications(b3PluginContext* context);
  B3_SHARED_API int processClientCommands(b3PluginContext* context);
#ifdef __cplusplus
};
#endif

#endif

