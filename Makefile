LIBS := -lmraa -lboost_program_options

CPP_SRCS += \
src/SFE_LSM9DS0.cpp \
src/still.cpp 

OBJS += \
src/SFE_LSM9DS0.o \
src/still.o 

OUT = still

src/%.o: src/%.cpp
	g++ -m32 -I"include" -c -o "$@" "$<"

# All Target
all: still

# Tool invocations
$(OUT): $(OBJS) $(USER_OBJS)
	g++ -m32 -o $(OUT) $(OBJS) $(LIBS)

# Other Targets
clean:
	rm `ls $(OUT) $(OBJS) 2>/dev/null` 2>/dev/null || true

.PHONY: all clean
.SECONDARY:

