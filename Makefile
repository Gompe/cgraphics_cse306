CXX=g++
CXXFLAGS=-std=c++17 -O3

EXE=render
ODIR=build

SRCS = $(wildcard *.cpp)
DEPS = $(wildcard *.h)
_OBJS = $(subst .cpp,.o,$(SRCS))
OBJS = $(patsubst %,$(ODIR)/%,$(_OBJS))

LIBS=-fopenmp

RM=rm -f

$(ODIR)/%.o: %.cpp $(DEPS)
	$(CXX) -c -o $@ $< $(CXXFLAGS)

render: $(SRCS) $(DEPS)
	$(CXX) -o render $(SRCS) $(CXXFLAGS) $(LIBS)

.PHONY: incremental
incremental: $(OBJS)
	$(CXX) -o render $^ $(CXXFLAGS) $(LIBS)

.PHONY: clean
clean:
	$(RM) $(OBJS) $(EXE)