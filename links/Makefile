
TARGETS :=
TARGETS += pr2.urdf
TARGETS += pr2.srdf
TARGETS += pr2.urdf.xacro

all: pr2.urdf pr2.srdf pr2.urdf.xacro

PR2_DESCRIPTION := $(shell rospack find pr2_description)
PR2_MOVEIT_CONFIG := $(shell rospack find pr2_moveit_config)

pr2.urdf: ${PR2_DESCRIPTION}/robots/pr2.urdf.xacro pr2.urdf.dep
	rosrun xacro xacro.py $< > $@

pr2.urdf.dep: Makefile
	rm -f pr2.urdf $@
	find ${PR2_DESCRIPTION} -name '*.xacro' \
		| sed -e 's|.*|$@: &|' \
		> $@

-include pr2.urdf.dep

pr2.srdf: ${PR2_MOVEIT_CONFIG}/config/pr2.srdf
	ln -sf $< $@

pr2.urdf.xacro: ${PR2_DESCRIPTION}/robots/pr2.urdf.xacro
	ln -sf $< $@


.PHONY: force
force:

clean:
	rm -f ${TARGETS} pr2.urdf.dep
	
