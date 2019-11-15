
/* Proof-of-concept plugin using the Bullet plugin API */

#include "testPlugin.h"
#include <SharedMemory/PhysicsServerExample.h>

typedef struct global {
	btScalar x;
	btScalar y;
	btScalar z;
} global_t;

/* Plugin functions */
#ifdef __cplusplus
extern "C" {
#endif

B3_SHARED_API int initPlugin(b3PluginContext* context)
{
  fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, __FUNCTION__);
  global_t* g = new global_t();
	g->x = 1;
	g->y = 2;
	g->z = 3;
	context->m_userPointer = g;
	return SHARED_MEMORY_MAGIC_NUMBER;
}

B3_SHARED_API int preTickPluginCallback(b3PluginContext* context)
{
  global_t* g = (global_t*)context->m_userPointer;
	return 0;
}

B3_SHARED_API int postTickPluginCallback(b3PluginContext* context)
{
  global_t* g = (global_t*)context->m_userPointer;
	return 0;
}

B3_SHARED_API int executePluginCommand(b3PluginContext* context, const b3PluginArguments* arguments)
{
  global_t* g = (global_t*)context->m_userPointer;
	fprintf(stderr, "testPlugin: argument: %s\n", arguments->m_text);
	for (int i = 0; i < arguments->m_numInts; ++i) {
		fprintf(stderr, "testPlugin: int arg[%d]: %d\n", i, arguments->m_ints[i]);
	}
	for (int i = 0; i < arguments->m_numFloats; ++i) {
		fprintf(stderr, "testPlugin: float arg[%d]: %f\n", i, arguments->m_floats[i]);
	}
	return -1;
}

B3_SHARED_API int processNotifications(b3PluginContext* context)
{
  return 0;
}

B3_SHARED_API int processClientCommands(b3PluginContext* context)
{
  return 0;
}

B3_SHARED_API void exitPlugin(b3PluginContext* context)
{
  fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, __FUNCTION__);
  global_t* g = (global_t*)context->m_userPointer;
  if (g) {
    delete g;
  }
}

#ifdef __cplusplus
}
#endif

