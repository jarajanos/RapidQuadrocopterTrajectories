TARGET := libquad-trajectories.a
OBJDIR := obj
LIBDIR := lib

SOURCES := $(wildcard C++/*.cpp)
SOURCES := $(SOURCES:C++/%=%) 
OBJECTS := $(patsubst %.c,%.o, $(patsubst %.cpp,%.o,$(SOURCES)))

CXXFLAGS := -std=c++17
CXX := g++
AR := ar rcu

LIBTAR := $(LIBDIR)/$(TARGET)
LIBOBJS := $(addprefix $(OBJDIR)/, $(addprefix $(LIBDIR)/, $(OBJECTS)))
LIBFLAGS := $(CXXFLAGS) -O3 -fno-math-errno

.PHONY: prep clean all

all: prep $(LIBTAR)

$(LIBTAR): $(LIBOBJS)
	$(AR) $(LIBTAR) $(LIBOBJS)

$(OBJDIR)/$(LIBDIR)/%.o: C++/%.cpp
	$(CXX) $(LIBFLAGS) -c $< -o $@

prep:
	@echo "Creating directories..."
	@mkdir -p $(OBJDIR)
	@mkdir -p $(OBJDIR)/$(LIBDIR)
	@mkdir -p $(LIBDIR)

clean:
	@echo "Cleaning..."
	@rm -rf $(OBJDIR)
	@rm -rf $(LIBDIR)
