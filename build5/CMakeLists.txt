cmake_minimum_required(VERSION 4.0)
project(formula1)

set(CMAKE_CXX_STANDARD 20)

# Укажите полный путь к SFML
set(SFML_PATH "D:/SFML-3.0.2")

# Включаем заголовочные файлы
include_directories(${SFML_PATH}/include)

# Добавляем путь к библиотекам
link_directories(${SFML_PATH}/lib)

# Создаем исполняемый файл
add_executable(formula1
        main.cpp
        F1_Physics_build_3.cpp
        F1_Physics_build_2.h
)

# Связываем с SFML библиотеками
target_link_libraries(formula1
        sfml-graphics
        sfml-window
        sfml-system
)

# Копирование DLL файлов
add_custom_command(TARGET formula1 POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${SFML_PATH}/bin/sfml-graphics-3.dll"
        $<TARGET_FILE_DIR:formula1>
)
add_custom_command(TARGET formula1 POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${SFML_PATH}/bin/sfml-window-3.dll"
        $<TARGET_FILE_DIR:formula1>
)
add_custom_command(TARGET formula1 POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${SFML_PATH}/bin/sfml-system-3.dll"
        $<TARGET_FILE_DIR:formula1>
)