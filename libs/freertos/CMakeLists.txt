cmake_minimum_required (VERSION 3.0)

set(FREERTOS_CONFIG_FILE_DIRECTORY .)

include_directories (include/)
include_directories (port/)

#croutine.c 
#event_groups.c list.c queue.c stream_buffer.c tasks.c timers.c
#freertos_kernel -- это имя цели!!! Не связано напрямую с именем папки!

add_library (freertos_kernel
	STATIC
	timers.c 
	tasks.c
	stream_buffer.c 
	queue.c 
	#mpu_wrappers.c
	list.c
	event_groups.c
	croutine.c
	port/port.c
	MemMang/heap_4.c
)

target_include_directories (freertos_kernel
	PUBLIC include port ${FREERTOS_CONFIG_FILE_DIRECTORY} )
