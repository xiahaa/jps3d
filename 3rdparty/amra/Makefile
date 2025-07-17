SRC = $(filter-out src/amra/CostConvergenceTerminationCondition.cpp, $(wildcard src/amra/*.cpp \
		src/smpl/*.cpp src/plan_2d.cpp src/smpl/console/*.cpp \
		src/smpl/unicycle/*.cpp))

OBJ = $(SRC:.cpp=.o)

D_INCLUDES = -I/usr/include -I/opt/local/include -I./include/ \
				-I/usr/include/eigen3/
D_LIBS = -L/usr/local/lib -L./lib \
			-lboost_system -lboost_filesystem -lboost_program_options

CC = g++
CFLAGS = -std=c++14 -pedantic -Wall -Wno-strict-aliasing -Wno-long-long \
		 -Wno-deprecated -Wno-deprecated-declarations -Wno-unused-result -Wno-c++14-extensions -fPIC
FAST_CFLAGS = -O3 -DNDEBUG
DEV_CFLAGS = -g -ggdb -O0
PROFILE_CFLAGS = -g -ggdb -O

ifeq ("$(findstring Darwin, "$(shell uname -s)")", "Darwin")
  CFLAGS += -DOS_MAC
else
  ifeq ("$(findstring Linux, "$(shell uname -s)")", "Linux")
    D_LIBS += -lrt
  endif
endif


.PHONY: all
all: dev

.PHONY: fast
fast: CFLAGS += $(FAST_CFLAGS) $(D_INCLUDES)
fast: main

.PHONY: dev
dev: CFLAGS += $(DEV_CFLAGS) $(D_INCLUDES)
dev: main

.PHONY: profile
profile: CFLAGS += $(PROFILE_CFLAGS) $(D_INCLUDES)
profile: main

.PHONY: tags
tags:
	ctags -R .

.PHONY: clean
clean:
	@rm -rf ./bin/*
	@-$(RM) ./lib/*
	@-$(RM) $(OBJ:.o=.d)
	@-$(RM) $(OBJ)

.PHONY: makedirs
makedirs:
	@echo "### Creating output directories ###"
	@$(shell mkdir ./bin)
	@$(shell mkdir ./lib)

.PHONY: AMRA
AMRA: makedirs $(OBJ)
	@echo "###  Archiving object files ###"
	@echo "sources "$(SRC)
	ar -crs lib/lib$(@).a $(OBJ)

.PHONY: main
main: makedirs AMRA
	@echo "linking..."
# 	$(CC) main.cpp -o ./bin/main -lAMRA $(CFLAGS) $(D_LIBS) $(D_INCLUDES)

.PHONY: python
python: AMRA
	@echo "### Building Python module ###"
	$(CC) -shared -std=c++14 -fPIC `python3 -m pybind11 --includes` python/wrapper.cpp -o AMRA`python3-config --extension-suffix` -L./lib -lAMRA $(CFLAGS) $(D_LIBS) $(D_INCLUDES)

-include $(OBJ:.o=.d)

$(OBJ):
	@echo "compiling..."
	$(CC) -c $(@:.o=.cpp) -MM -MP -MT $(dir $(@))$(notdir $(@)) -MF $(@:.o=.d) $(CFLAGS)
	$(CC) -c $(@:.o=.cpp) -o $(@) $(CFLAGS)
