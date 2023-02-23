vpath %.cpp ./src
vpath %.h ./include

.PHONY: all clean clean_cache mytest
all : # 默认目标

#第三方库
# LINK_LIB= -lopencv_core -lopencv_imgproc -lopencv_highgui
LINK_LIB= `pkg-config --cflags --libs opencv-3.3.1-dev`

CXXINCLUDES = -I ./include $(LINK_LIB)	#包含的用户头文件路径
SRCS_WITH_PATH = $(wildcard ./src/*.cpp)	#搜索./src/*.cpp中所有CPP文件名赋值给SRCS_WITH_PATH
SRCS = $(SRCS_WITH_PATH:./src/%.cpp=%.cpp)	#去掉SRCS_WITH_PATH中所有路径名

DEPS = $(SRCS:.cpp=.d)			#依赖头文件
DEPS_WITH_PATH = $(SRCS:%.cpp=./.d/%.d)

OBJS = $(SRCS:.cpp=.o)						#目标名
OBJS_WITH_PATH = $(SRCS:%.cpp=./compile/%.o)

#用户静态库和动态库，.a文件，是LINUX系统中的静态链接库文件。.so文件就是常说的动态链接库
# LINK_USR_LIB = ../../lib/tsi.a
# LINK_USR_SO = -L ../../lib -Wl,-rpath=../lib -ltest

#目标名称和位置
TARGET_NAME = main
OUTDIR = ../release/

ifeq ($(BUILD_VER), DEBUG)
CXXFLAGS += -g3
# LINK_LIB := ../../lib/tsiD.a
# LINK_USR_SO := -L ../../lib -Wl,-rpath=../lib -ltestD
TARGET_NAME := main_D
OUTDIR := ../debug/
endif

ifeq ($(BUILD_VER), RELEASE)
CXXFLAGS += -O2
endif

#生成依赖关系,保证修改.h时也会重新编译相关.cpp
-include $(DEPS_WITH_PATH)

#生成 .d文件， > ./$@ 表示生成的路径位置和名称
%.d:$(SRCS)
	set -e;\
	$(RM) $@;\
	$(CXX) $(CXXINCLUDES) -M $< > $@;

%.o:%.cpp
	$(CXX) $(CXXFLAGS) $(CXXINCLUDES) -c $< -o ./compile/$@

all:$(TARGET_NAME)

$(TARGET_NAME):$(OBJS)
	#$(CXX) -o $(OUTDIR)$(TARGET_NAME) $(OBJS_WITH_PATH) $(LINK_LIB) $(LINK_USR_SO)
	$(CXX) -o $(OUTDIR)$(TARGET_NAME) $(OBJS_WITH_PATH) $(LINK_LIB)

clean:
	$(RM) $(OUTDIR)$(TARGET_NAME) $(OBJS_WITH_PATH) $(DEPS_WITH_PATH)

clean_cache:
	$(RM) $(OBJS_WITH_PATH) $(DEPS_WITH_PATH)

mytest:
	@echo CXX=$(CXX)
	@echo SRCS_WITH_PATH=$(SRCS_WITH_PATH)
	@echo SRCS=$(SRCS)
	@echo OBJS=$(OBJS)
	@echo RM=$(RM)
	@echo BUILD_VER=$(BUILD_VER)
	@echo DEPS=$(DEPS)
	@echo DEPS_WITH_PATH=$(DEPS_WITH_PATH)