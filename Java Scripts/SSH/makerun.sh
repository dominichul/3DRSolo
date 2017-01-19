#/bin/sh

# Compiles and runs the ssh java app
javac -cp jsch-0.1.54.jar JavaSSH.java
java -cp .:jsch-0.1.54.jar JavaSSH