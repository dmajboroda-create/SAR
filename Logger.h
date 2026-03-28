#pragma once

/**
 * @file Logger.h
 * @brief Налаштування та ініціалізація системи логування на базі spdlog.
 *
 * Підтримує два обробники:
 * - Console sink: виводить кольорові логи в консоль
 * - Rotating file sink: записує логи у файл з ротацією за розміром
 *
 * Конфігурація завантажується з logger_config.json без перекомпіляції.
 *
 * ## Рівні логування:
 * - DEBUG   — детальна діагностична інформація (крок інтегрування, параметри)
 * - INFO    — ключові події (запуск, завершення, збереження файлу)
 * - WARNING — нестандартні ситуації (параметри за замовчуванням)
 * - ERROR   — помилки з можливістю продовження (помилка файлу)
 * - CRITICAL — критичні помилки, що зупиняють роботу (T == 0)
 *
 * @author Мажборода Д. М.
 * @date 2025
 */

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/basic_file_sink.h>

#include <fstream>
#include <string>
#include <vector>
#include <memory>
#include <filesystem>
#include <stdexcept>

// Простий JSON-парсер для конфігурації (без зовнішніх залежностей)
namespace LogConfig {
    struct Config {
        std::string log_level    = "info";
        std::string log_file     = "logs/sar_simulation.log";
        bool log_to_console      = true;
        bool log_to_file         = true;
        int  max_file_size_mb    = 10;
        int  max_files           = 5;
        std::string pattern      = "[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [%s:%#] %v";
    };

    /**
     * @brief Читає рядкове значення з мінімального JSON.
     */
    inline std::string extractString(const std::string& json, const std::string& key) {
        std::string search = "\"" + key + "\"";
        size_t pos = json.find(search);
        if (pos == std::string::npos) { return ""; }
        pos = json.find(':', pos);
        if (pos == std::string::npos) { return ""; }
        pos = json.find('"', pos);
        if (pos == std::string::npos) { return ""; }
        size_t end = json.find('"', pos + 1);
        if (end == std::string::npos) { return ""; }
        return json.substr(pos + 1, end - pos - 1);
    }

    /**
     * @brief Читає булеве значення з мінімального JSON.
     */
    inline bool extractBool(const std::string& json, const std::string& key, bool defaultVal) {
        std::string search = "\"" + key + "\"";
        size_t pos = json.find(search);
        if (pos == std::string::npos) { return defaultVal; }
        pos = json.find(':', pos);
        if (pos == std::string::npos) { return defaultVal; }
        size_t truePos  = json.find("true",  pos);
        size_t falsePos = json.find("false", pos);
        size_t nextKey  = json.find('"', pos + 1);
        if (truePos  != std::string::npos && (nextKey == std::string::npos || truePos  < nextKey)) { return true; }
        if (falsePos != std::string::npos && (nextKey == std::string::npos || falsePos < nextKey)) { return false; }
        return defaultVal;
    }

    /**
     * @brief Читає ціле число з мінімального JSON.
     */
    inline int extractInt(const std::string& json, const std::string& key, int defaultVal) {
        std::string search = "\"" + key + "\"";
        size_t pos = json.find(search);
        if (pos == std::string::npos) { return defaultVal; }
        pos = json.find(':', pos);
        if (pos == std::string::npos) { return defaultVal; }
        while (pos < json.size() && (json[pos] == ':' || json[pos] == ' ')) { ++pos; }
        try { return std::stoi(json.substr(pos)); }
        catch (...) { return defaultVal; }
    }

    /**
     * @brief Завантажує конфігурацію логера з JSON-файлу.
     *
     * Якщо файл не знайдено — повертає налаштування за замовчуванням.
     * Це дозволяє змінювати рівень логування без перекомпіляції.
     *
     * @param path Шлях до файлу конфігурації
     * @return Структура Config з параметрами логера
     */
    inline Config load(const std::string& path = "logger_config.json") {
        Config cfg;
        std::ifstream file(path);
        if (!file.is_open()) {
            return cfg; // повертаємо defaults
        }
        std::string json((std::istreambuf_iterator<char>(file)),
                          std::istreambuf_iterator<char>());

        std::string level = extractString(json, "log_level");
        if (!level.empty()) { cfg.log_level = level; }

        std::string logFile = extractString(json, "log_file");
        if (!logFile.empty()) { cfg.log_file = logFile; }

        cfg.log_to_console = extractBool(json, "log_to_console", cfg.log_to_console);
        cfg.log_to_file    = extractBool(json, "log_to_file",    cfg.log_to_file);
        cfg.max_file_size_mb = extractInt(json, "max_file_size_mb", cfg.max_file_size_mb);
        cfg.max_files        = extractInt(json, "max_files",        cfg.max_files);

        std::string pattern = extractString(json, "pattern");
        if (!pattern.empty()) { cfg.pattern = pattern; }

        return cfg;
    }
}

/**
 * @brief Ініціалізує глобальний логер spdlog з конфігурації.
 *
 * Налаштовує:
 * - Console sink (кольоровий вивід у термінал)
 * - Rotating file sink (ротація за розміром, max_files архівів)
 *
 * Рівень логування визначається з logger_config.json,
 * що дозволяє змінювати його без перекомпіляції.
 *
 * @param configPath Шлях до файлу конфігурації (за замовчуванням "logger_config.json")
 */
inline void initLogger(const std::string& configPath = "logger_config.json") {
    auto cfg = LogConfig::load(configPath);

    // Створюємо папку для логів якщо не існує
    std::filesystem::path logPath(cfg.log_file);
    if (logPath.has_parent_path()) {
        std::filesystem::create_directories(logPath.parent_path());
    }

    std::vector<spdlog::sink_ptr> sinks;

    // Console sink — кольоровий вивід
    if (cfg.log_to_console) {
        auto consoleSink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        consoleSink->set_pattern(cfg.pattern);
        sinks.push_back(consoleSink);
    }

    // Rotating file sink — ротація за розміром
    if (cfg.log_to_file) {
        size_t maxSize = static_cast<size_t>(cfg.max_file_size_mb) * 1024 * 1024;
        auto fileSink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
            cfg.log_file, maxSize, cfg.max_files);
        fileSink->set_pattern(cfg.pattern);
        sinks.push_back(fileSink);
    }

    auto logger = std::make_shared<spdlog::logger>("SAR", sinks.begin(), sinks.end());

    // Встановлюємо рівень логування з конфігурації
    spdlog::level::level_enum level = spdlog::level::info;
    if      (cfg.log_level == "debug"    || cfg.log_level == "DEBUG")    { level = spdlog::level::debug; }
    else if (cfg.log_level == "info"     || cfg.log_level == "INFO")     { level = spdlog::level::info; }
    else if (cfg.log_level == "warning"  || cfg.log_level == "WARNING")  { level = spdlog::level::warn; }
    else if (cfg.log_level == "error"    || cfg.log_level == "ERROR")    { level = spdlog::level::err; }
    else if (cfg.log_level == "critical" || cfg.log_level == "CRITICAL") { level = spdlog::level::critical; }

    logger->set_level(level);
    logger->flush_on(spdlog::level::err); // миттєвий flush при помилках

    spdlog::set_default_logger(logger);
    spdlog::set_level(level);
}