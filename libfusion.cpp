#include "libfusion.h"
#include "sensor_fusion.h"
void* fusion_init(void){
	return (void*)(new SensorFusion()); 
}

void  fusion_enable_sensor(void *h, int sensor_type, bool enable){
	SensorFusion* fusion = (SensorFusion*)h;
	if(!fusion)
		return;
	if(sensor_type == TYPE_ROTATION_VECTOR){
		fusion->activate(FUSION_9AXIS, !!enable);
	}else if(sensor_type == TYPE_GAME_ROTATION_VECTOR){
		fusion->activate(FUSION_NOMAG, !!enable);
	}else if(sensor_type == TYPE_GEOMAGNETIC_ROTATION_VECTOR){
		fusion->activate(FUSION_NOGYRO, !!enable);
	}
}

bool  fusion_is_enabled(void *h, int sensor_type){
	SensorFusion* fusion = (SensorFusion*)h;
	if(!fusion)
		return false;
	if(sensor_type == TYPE_ROTATION_VECTOR){
		return fusion->isEnable(FUSION_9AXIS);
	}else if(sensor_type == TYPE_GAME_ROTATION_VECTOR){
		return fusion->isEnable(FUSION_NOMAG);
	}else if(sensor_type == TYPE_GEOMAGNETIC_ROTATION_VECTOR){
		return fusion->isEnable(FUSION_NOGYRO);
	}
	return false;
}


void fusion_process(void* h, const sf_data* data){
	SensorFusion* fusion = (SensorFusion*)h;
	if(!fusion)
		return;

	fusion->process(*data);
}

void fusion_get_data(void* h, sf_data* data){
	SensorFusion* fusion = (SensorFusion*)h;
	if(!fusion)
		return;

	if(data->type == TYPE_GAME_ROTATION_VECTOR){
		vec4_t q = fusion->getAttitude(FUSION_NOMAG);	
		data->rv.i = q.x;
		data->rv.j = q.y;
		data->rv.k = q.z;
		data->rv.w = q.w;
	}else if(data->type == TYPE_ROTATION_VECTOR){
		vec4_t q = fusion->getAttitude(FUSION_9AXIS);	
		data->rv.i = q.x;
		data->rv.j = q.y;
		data->rv.k = q.z;
		data->rv.w = q.w;
	}else if(data->type == TYPE_GEOMAGNETIC_ROTATION_VECTOR){
		vec4_t q = fusion->getAttitude(FUSION_NOGYRO);	
		data->rv.i = q.x;
		data->rv.j = q.y;
		data->rv.k = q.z;
		data->rv.w = q.w;
	}
	data->timeStamp = fusion->getTimeStamp(data->type);
}

void  fusion_release(void *h){
	SensorFusion* fusion = (SensorFusion*)h;
	if(!fusion)
		return;


	free(fusion);
}


void fusion_dump(void* h){
	SensorFusion* fusion = (SensorFusion*)h;
	if(!fusion)
		return;

	fusion->dump();
}
