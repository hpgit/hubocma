# Compiler flags...
#CPP_COMPILER = g++
#C_COMPILER = gcc
CPP_COMPILER = clang++
C_COMPILER = clang++

# Include paths...
Release_Include_Path=-I"../include" 

# Library paths...
Release_Library_Path=

# Additional libraries...
Release_Libraries=

# Preprocessor definitions...
Release_Preprocessor_Definitions=-D GCC_BUILD -D NDEBUG -D _LIB 
#-fopenmp

# Implictly linked object files...
Release_Implicitly_Linked_Objects=

# Compiler flags...
Release_Compiler_Flags=-arch=x86_64 -O2 -stdlib=libstdc++

# Builds all configurations for this project...
.PHONY: build_all_configurations
build_all_configurations: Release 

# Builds the Release configuration...
.PHONY: Release
Release: create_folders tmp/Win32/gccRelease/gbody.o tmp/Win32/gccRelease/gconstraint.o tmp/Win32/gccRelease/gconstraint_jointloop.o tmp/Win32/gccRelease/gelement.o tmp/Win32/gccRelease/gjoint.o tmp/Win32/gccRelease/gjoint_composite.o tmp/Win32/gccRelease/gjoint_fixed.o tmp/Win32/gccRelease/gjoint_free.o tmp/Win32/gccRelease/gjoint_planar.o tmp/Win32/gccRelease/gjoint_prismatic.o tmp/Win32/gccRelease/gjoint_revolute.o tmp/Win32/gccRelease/gjoint_spherical.o tmp/Win32/gccRelease/gjoint_translational.o tmp/Win32/gccRelease/gjoint_universal.o tmp/Win32/gccRelease/gspringdamper.o tmp/Win32/gccRelease/gsystem.o tmp/Win32/gccRelease/gsystem_constrained.o tmp/Win32/gccRelease/gsystem_ik.o tmp/Win32/gccRelease/liegroup.o tmp/Win32/gccRelease/rmatrix3j.o
	ar rcs ../lib/libgear.a tmp/Win32/gccRelease/gbody.o tmp/Win32/gccRelease/gconstraint.o tmp/Win32/gccRelease/gconstraint_jointloop.o tmp/Win32/gccRelease/gelement.o tmp/Win32/gccRelease/gjoint.o tmp/Win32/gccRelease/gjoint_composite.o tmp/Win32/gccRelease/gjoint_fixed.o tmp/Win32/gccRelease/gjoint_free.o tmp/Win32/gccRelease/gjoint_planar.o tmp/Win32/gccRelease/gjoint_prismatic.o tmp/Win32/gccRelease/gjoint_revolute.o tmp/Win32/gccRelease/gjoint_spherical.o tmp/Win32/gccRelease/gjoint_translational.o tmp/Win32/gccRelease/gjoint_universal.o tmp/Win32/gccRelease/gspringdamper.o tmp/Win32/gccRelease/gsystem.o tmp/Win32/gccRelease/gsystem_constrained.o tmp/Win32/gccRelease/gsystem_ik.o tmp/Win32/gccRelease/liegroup.o tmp/Win32/gccRelease/rmatrix3j.o

# Compiles file gbody.cpp for the Release configuration...
-include tmp/Win32/gccRelease/gbody.d
tmp/Win32/gccRelease/gbody.o: gbody.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c gbody.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/gbody.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM gbody.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/gbody.d

# Compiles file gconstraint.cpp for the Release configuration...
-include tmp/Win32/gccRelease/gconstraint.d
tmp/Win32/gccRelease/gconstraint.o: gconstraint.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c gconstraint.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/gconstraint.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM gconstraint.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/gconstraint.d

# Compiles file gconstraint_jointloop.cpp for the Release configuration...
-include tmp/Win32/gccRelease/gconstraint_jointloop.d
tmp/Win32/gccRelease/gconstraint_jointloop.o: gconstraint_jointloop.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c gconstraint_jointloop.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/gconstraint_jointloop.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM gconstraint_jointloop.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/gconstraint_jointloop.d

# Compiles file gelement.cpp for the Release configuration...
-include tmp/Win32/gccRelease/gelement.d
tmp/Win32/gccRelease/gelement.o: gelement.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c gelement.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/gelement.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM gelement.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/gelement.d

# Compiles file gjoint.cpp for the Release configuration...
-include tmp/Win32/gccRelease/gjoint.d
tmp/Win32/gccRelease/gjoint.o: gjoint.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c gjoint.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/gjoint.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM gjoint.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/gjoint.d

# Compiles file gjoint_composite.cpp for the Release configuration...
-include tmp/Win32/gccRelease/gjoint_composite.d
tmp/Win32/gccRelease/gjoint_composite.o: gjoint_composite.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c gjoint_composite.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/gjoint_composite.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM gjoint_composite.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/gjoint_composite.d

# Compiles file gjoint_fixed.cpp for the Release configuration...
-include tmp/Win32/gccRelease/gjoint_fixed.d
tmp/Win32/gccRelease/gjoint_fixed.o: gjoint_fixed.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c gjoint_fixed.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/gjoint_fixed.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM gjoint_fixed.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/gjoint_fixed.d

# Compiles file gjoint_free.cpp for the Release configuration...
-include tmp/Win32/gccRelease/gjoint_free.d
tmp/Win32/gccRelease/gjoint_free.o: gjoint_free.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c gjoint_free.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/gjoint_free.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM gjoint_free.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/gjoint_free.d

# Compiles file gjoint_planar.cpp for the Release configuration...
-include tmp/Win32/gccRelease/gjoint_planar.d
tmp/Win32/gccRelease/gjoint_planar.o: gjoint_planar.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c gjoint_planar.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/gjoint_planar.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM gjoint_planar.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/gjoint_planar.d

# Compiles file gjoint_prismatic.cpp for the Release configuration...
-include tmp/Win32/gccRelease/gjoint_prismatic.d
tmp/Win32/gccRelease/gjoint_prismatic.o: gjoint_prismatic.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c gjoint_prismatic.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/gjoint_prismatic.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM gjoint_prismatic.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/gjoint_prismatic.d

# Compiles file gjoint_revolute.cpp for the Release configuration...
-include tmp/Win32/gccRelease/gjoint_revolute.d
tmp/Win32/gccRelease/gjoint_revolute.o: gjoint_revolute.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c gjoint_revolute.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/gjoint_revolute.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM gjoint_revolute.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/gjoint_revolute.d

# Compiles file gjoint_spherical.cpp for the Release configuration...
-include tmp/Win32/gccRelease/gjoint_spherical.d
tmp/Win32/gccRelease/gjoint_spherical.o: gjoint_spherical.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c gjoint_spherical.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/gjoint_spherical.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM gjoint_spherical.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/gjoint_spherical.d

# Compiles file gjoint_translational.cpp for the Release configuration...
-include tmp/Win32/gccRelease/gjoint_translational.d
tmp/Win32/gccRelease/gjoint_translational.o: gjoint_translational.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c gjoint_translational.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/gjoint_translational.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM gjoint_translational.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/gjoint_translational.d

# Compiles file gjoint_universal.cpp for the Release configuration...
-include tmp/Win32/gccRelease/gjoint_universal.d
tmp/Win32/gccRelease/gjoint_universal.o: gjoint_universal.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c gjoint_universal.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/gjoint_universal.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM gjoint_universal.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/gjoint_universal.d

# Compiles file gspringdamper.cpp for the Release configuration...
-include tmp/Win32/gccRelease/gspringdamper.d
tmp/Win32/gccRelease/gspringdamper.o: gspringdamper.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c gspringdamper.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/gspringdamper.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM gspringdamper.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/gspringdamper.d

# Compiles file gsystem.cpp for the Release configuration...
-include tmp/Win32/gccRelease/gsystem.d
tmp/Win32/gccRelease/gsystem.o: gsystem.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c gsystem.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/gsystem.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM gsystem.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/gsystem.d

# Compiles file gsystem_constrained.cpp for the Release configuration...
-include tmp/Win32/gccRelease/gsystem_constrained.d
tmp/Win32/gccRelease/gsystem_constrained.o: gsystem_constrained.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c gsystem_constrained.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/gsystem_constrained.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM gsystem_constrained.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/gsystem_constrained.d

# Compiles file gsystem_ik.cpp for the Release configuration...
-include tmp/Win32/gccRelease/gsystem_ik.d
tmp/Win32/gccRelease/gsystem_ik.o: gsystem_ik.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c gsystem_ik.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/gsystem_ik.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM gsystem_ik.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/gsystem_ik.d

# Compiles file liegroup.cpp for the Release configuration...
-include tmp/Win32/gccRelease/liegroup.d
tmp/Win32/gccRelease/liegroup.o: liegroup.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c liegroup.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/liegroup.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM liegroup.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/liegroup.d

# Compiles file rmatrix3j.cpp for the Release configuration...
-include tmp/Win32/gccRelease/rmatrix3j.d
tmp/Win32/gccRelease/rmatrix3j.o: rmatrix3j.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c rmatrix3j.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/rmatrix3j.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM rmatrix3j.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/rmatrix3j.d

# Compiles file vpSingleSystem.cpp for the Release configuration...
-include tmp/Win32/gccRelease/vpSingleSystem.d
tmp/Win32/gccRelease/vpSingleSystem.o: vpSingleSystem.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c vpSingleSystem.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/vpSingleSystem.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM vpSingleSystem.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/vpSingleSystem.d

# Creates the intermediate and output folders for each configuration...
.PHONY: create_folders
create_folders:
	mkdir -p tmp/Win32/gccRelease/source
	mkdir -p ../lib

# Cleans intermediate and output files (objects, libraries, executables)...
.PHONY: clean
clean:
	rm -f tmp/Win32/gccRelease/*.o
	rm -f tmp/Win32/gccRelease/*.d
	rm -f ../lib/*.a
	rm -f ../lib/*.so
	rm -f ../lib/*.dll
	rm -f ../lib/*.exe
