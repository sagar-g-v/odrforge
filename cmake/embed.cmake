# Embed a text file as a C string: cmake -DIN=<file> -DOUT=<cpp> -DNAME=<symbol> -P embed.cmake
file(READ "${IN}" content HEX)
string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," bytes "${content}")
file(WRITE "${OUT}" "// generated from ${IN} - do not edit\nnamespace odrforge {\nstatic const char ${NAME}Data[] = {${bytes}0x00};\nconst char *${NAME} = ${NAME}Data;\n}\n")
