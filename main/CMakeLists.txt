set(srcs "main.cpp"
    "bmi160.c"
    )

idf_component_register(SRCS ${srcs} INCLUDE_DIRS ".")