ifndef MODULE_SENSOR_OPTIMIZATION
	MODULE_SENSOR_OPTIMIZATION = -g3 -O0
endif

MODULE_SENSOR_CPP_FILE			:= $(shell find module_sensor/ -maxdepth 3 -type f -name "*.cpp" )
MODULE_SENSOR_DIR				:= $(shell find module_sensor/ -maxdepth 3 -type d -name "*" )

MODULE_SENSOR_DIR				+= $(shell find module_mc_hardware_interfaces/ -maxdepth 10 -type d -name "*" )

MODULE_SENSOR_PATH				:= $(addprefix -I, $(MODULE_SENSOR_DIR))
MODULE_SENSOR_OBJ_FILE			:= $(addprefix build/obj/, $(MODULE_SENSOR_CPP_FILE))
MODULE_SENSOR_OBJ_FILE			:= $(patsubst %.cpp, %.o, $(MODULE_SENSOR_OBJ_FILE))


build/obj/module_sensor/%.o:	module_sensor/%.cpp
	@echo [CPP] $<
	@mkdir -p $(dir $@)
	@$(CPP) $(CPP_FLAGS) $(DEFINE_PROJ) $(USER_OS_PATH) $(USER_CFG_PATH) $(MODULE_SENSOR_PATH) $(MODULE_SENSOR_OPTIMIZATION) -c $< -o $@

# Добавляем к общим переменным проекта.
PROJECT_PATH			+= $(MODULE_SENSOR_PATH)
PROJECT_OBJ_FILE		+= $(MODULE_SENSOR_OBJ_FILE)