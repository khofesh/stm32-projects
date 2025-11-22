#pragma once
#include <pqxx/pqxx>
#include <string>
#include <optional>

struct Measurement {
    float pm1_0;
    float pm2_5;
    float pm4_0;
    float pm10;
    std::optional<float> temperature;
    std::optional<float> humidity;
    std::optional<int> voc_index;
    std::optional<int> nox_index;
};

class DbPostgresql {
private:
    pqxx::connection conn;

public:
    // Constructor with connection string
    DbPostgresql(const std::string& connection_string);
    ~DbPostgresql();
    
    // Insert a new measurement to the database
    void insert(const Measurement& m);
    
    // Check if connection is active
    bool isConnected() const;
};
