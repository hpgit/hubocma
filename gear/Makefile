# Builds all the projects in the solution...
#.PHONY: all_projects
#all_projects: gear
#all_projects: vpLib vpRenderer stacking ragdoll tutorial 

# Builds project 'gear'...
.PHONY: gear
vpLib: 
	make --directory="src/" --file=gear.makefile

## Builds project 'vpRenderer'...
#.PHONY: vpRenderer
#vpRenderer: vpLib 
#	make --directory="vpRenderer/" --file=vpRenderer.makefile

# Builds project 'stacking'...
.PHONY: stacking
stacking: vpRenderer vpLib 
	make --directory="Examples/stacking/" --file=stacking.makefile

# Builds project 'ragdoll'...
.PHONY: ragdoll
ragdoll: vpRenderer vpLib 
	make --directory="Examples/ragdoll/" --file=ragdoll.makefile

# Builds project 'tutorial'...
.PHONY: tutorial
tutorial: vpRenderer vpLib 
	make --directory="Examples/tutorial/" --file=tutorial.makefile

# Cleans all projects...
.PHONY: clean
clean:
	make --directory="src/" --file=gear.makefile clean
#	make --directory="vpRenderer/" --file=vpRenderer.makefile clean
#	make --directory="Examples/stacking/" --file=stacking.makefile clean
#	make --directory="Examples/ragdoll/" --file=ragdoll.makefile clean
#	make --directory="Examples/tutorial/" --file=tutorial.makefile clean

