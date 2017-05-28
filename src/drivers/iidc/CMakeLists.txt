file(GLOB iidc_SRCS *.cpp)

add_driver(NAME iidc OS Linux Windows Darwin DEFAULT_ON SRCS ${iidc_SRCS})
