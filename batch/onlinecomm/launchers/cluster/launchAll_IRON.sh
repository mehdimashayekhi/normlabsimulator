#!/bin/bash


# Initial settings
ECLIPSE_HOME=/home/iosu/Repast-Simphony-2.1/eclipse
JAVA_EXECUTABLE=java
JAVA_FLAGS=-client
ECLIPSE_PLUGINS=$ECLIPSE_HOME/plugins

WORKSPACE=/home/iosu/workspace
IRON_DIR=/home/iosu/workspace/NormSynthesisMachine
NORMLAB_DIR=$WORKSPACE/NormLabSimulators
PROJECT_DIR=$NORMLAB_DIR

# Prepare CLASSPATH
CLASSPATH=.

TMP_LIBS="libs_todelete.txt"
echo "" > $TMP_LIBS

# Add libraries to path (Eclipse, Repast)
ECLIPSE_PLUGINS_ALL_JARS=`find $ECLIPSE_PLUGINS/repast.simphony.* -type f -name '*.jar'`
ECLIPSE_PLUGINS_ALL_JARS=`find $ECLIPSE_PLUGINS/saf.core* -type f -name '*.jar'`:$ECLIPSE_PLUGINS_ALL_JARS
for file in $ECLIPSE_PLUGINS_ALL_JARS
do
  	echo "CLASSPATH=\$CLASSPATH:$file" >> $TMP_LIBS
done


# Add IRON libs
IRON_LIBS=`find $IRON_DIR/lib -type f -name '*.jar'`
for file in $IRON_LIBS
do
echo "CLASSPATH=\$CLASSPATH:$file" >> $TMP_LIBS
done

# Add Virtual Communities Simulation libs
PROJECT_LIBS=`find $PROJECT_DIR/lib -type f -name '*.jar'`
for file in $PROJECT_LIBS
do
echo "CLASSPATH=\$CLASSPATH:$file" >> $TMP_LIBS
done

# Add sources to path
echo "CLASSPATH=\$CLASSPATH:$IRON_DIR/bin" >> $TMP_LIBS
echo "CLASSPATH=\$CLASSPATH:$PROJECT_DIR/bin" >> $TMP_LIBS

# Format Libraries (sort...)
sort $TMP_LIBS > "$TMP_LIBS-sorted"
echo "export CLASSPATH=\$CLASSPATH" >> "$TMP_LIBS-sorted"
source "$TMP_LIBS-sorted"
export CLASSPATH=$CLASSPATH
rm $TMP_LIBS "$TMP_LIBS-sorted"

######################
# Launch experiments #
######################

for i in `seq 1 36`; do
	qsub IRON_Experiments_Launcher_$i.sh
done


