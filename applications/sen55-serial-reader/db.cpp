#include "db.hpp"
#include <iostream>
#include <stdexcept>

// Initialize connection to PostgreSQL/TimescaleDB
DbPostgresql::DbPostgresql(const std::string& connection_string)
    : conn(connection_string) {
    if (!conn.is_open()) {
        throw std::runtime_error("Failed to open database connection");
    }
    std::cout << "Connected to database: " << conn.dbname() << std::endl;
}

// connection is automatically closed by pqxx::connection
DbPostgresql::~DbPostgresql() {
    if (conn.is_open()) {
        std::cout << "Closing database connection" << std::endl;
    }
}

// Insert a new measurement to the database
void DbPostgresql::insert(const Measurement& m) {
    try {
        // Create a work transaction
        pqxx::work txn(conn);
        
        // query
        std::string query = 
            "INSERT INTO public.measurements "
            "(time, pm1_0, pm2_5, pm4_0, pm10, temperature, humidity, voc_index, nox_index) "
            "VALUES (NOW(), $1, $2, $3, $4, $5, $6, $7, $8)";
        
        // pqxx automatically handles NULL for std::optional when it's empty
        txn.exec_params(
            query,
            m.pm1_0,
            m.pm2_5,
            m.pm4_0,
            m.pm10,
            m.temperature,
            m.humidity,
            m.voc_index,
            m.nox_index
        );
        
        txn.commit();
        
    } catch (const std::exception& e) {
        std::cerr << "Database insert error: " << e.what() << std::endl;
        throw;
    }
}

// Check if connection is active
bool DbPostgresql::isConnected() const {
    return conn.is_open();
}
