qt_add_executable(LoRaTemperature
    main.cpp
    Sample.h
    AppConfig.h
    AppConfig.cpp
    CsvWriter.h
    CsvWriter.cpp
    ModbusWorker.h
    ModbusWorker.cpp
    ChartManager.h
    ChartManager.cpp
    MainWindow.h
    MainWindow.cpp
)

target_link_libraries(LoRaTemperature PRIVATE
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    Qt6::SerialPort
    Qt6::SerialBus
    Qt6::Charts
)

target_include_directories(LoRaTemperature PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

install(TARGETS LoRaTemperature
    RUNTIME DESTINATION bin
    BUNDLE DESTINATION .
)
