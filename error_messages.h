#pragma once
#include <string>
#include <map>

/**
 * @file error_messages.h
 * @brief Локалізовані повідомлення про помилки для кінцевого користувача.
 *
 * Реалізує локалізацію повідомлень про помилки на двох мовах:
 * - uk (українська) — за замовчуванням
 * - en (англійська)
 *
 * Мова визначається зі змінної оточення SAR_LANG без перекомпіляції:
 * @code
 * SET SAR_LANG=en
 * .\engine_simulation.exe
 * @endcode
 */

/**
 * @brief Структура для зберігання локалізованого повідомлення про помилку.
 */
struct ErrorMessage {
    std::string title;       ///< Заголовок помилки
    std::string cause;       ///< Причина виникнення
    std::string action;      ///< Дії користувача
    std::string contact;     ///< Контакт для зв'язку з розробником
};

/**
 * @brief Клас локалізованих повідомлень про помилки.
 *
 * Підтримує мови: uk (українська), en (англійська).
 * Мова визначається зі змінної оточення SAR_LANG.
 */
class ErrorMessages {
public:
    /**
     * @brief Повертає локалізоване повідомлення для заданого коду помилки.
     *
     * @param errorCode Код помилки (ERR-001, ERR-002, ERR-003, ERR-004)
     * @return Структура ErrorMessage з локалізованим текстом
     */
    static ErrorMessage get(const std::string& errorCode) {
        std::string lang = getLang();
        auto& messages = (lang == "en") ? en_messages : uk_messages;

        auto it = messages.find(errorCode);
        if (it != messages.end()) {
            return it->second;
        }
        return uk_messages.at("ERR-000");
    }

    /**
     * @brief Виводить повідомлення про помилку у стандартний вивід помилок.
     *
     * Формат повідомлення не містить технічних деталей —
     * призначений для кінцевого користувача.
     *
     * @param errorCode Код помилки
     */
    static void print(const std::string& errorCode) {
        auto msg = get(errorCode);
        std::cerr << "\n╔══════════════════════════════════════╗\n";
        std::cerr << "  [!] " << msg.title << "\n";
        std::cerr << "  Причина: " << msg.cause << "\n";
        std::cerr << "  Дія:     " << msg.action << "\n";
        std::cerr << "  Код:     " << errorCode << "\n";
        std::cerr << "  Контакт: " << msg.contact << "\n";
        std::cerr << "╚══════════════════════════════════════╝\n\n";
    }

private:
    /**
     * @brief Визначає мову зі змінної оточення SAR_LANG.
     * @return Код мови ("uk" або "en")
     */
    static std::string getLang() {
        const char* lang = std::getenv("SAR_LANG");
        if (lang && std::string(lang) == "en") { return "en"; }
        return "uk";
    }

    // Українські повідомлення
    static inline std::map<std::string, ErrorMessage> uk_messages = {
        {"ERR-000", {"Невідома помилка",
                     "Виникла непередбачена ситуація.",
                     "Перезапустіть програму.",
                     "majboroda@example.com"}},
        {"ERR-001", {"Помилка файлової системи",
                     "Неможливо створити файл simulation_results.csv.",
                     "Перевірте права доступу до поточної папки.",
                     "majboroda@example.com"}},
        {"ERR-002", {"Помилка параметрів системи",
                     "Параметр T (стала часу) дорівнює нулю.",
                     "Перезапустіть програму та введіть T > 0 (рекомендовано: 0.1).",
                     "majboroda@example.com"}},
        {"ERR-003", {"Помилка моделювання",
                     "Виникла непередбачена помилка під час інтегрування.",
                     "Перезапустіть програму. Якщо помилка повторюється — зверніться до розробника.",
                     "majboroda@example.com"}},
        {"ERR-004", {"Помилка візуалізації",
                     "Не вдалося запустити Python-скрипт visualize.py.",
                     "Запустіть вручну: python3 visualize.py",
                     "majboroda@example.com"}},
    };

    // English messages
    static inline std::map<std::string, ErrorMessage> en_messages = {
        {"ERR-000", {"Unknown error",
                     "An unexpected situation occurred.",
                     "Restart the program.",
                     "majboroda@example.com"}},
        {"ERR-001", {"File system error",
                     "Cannot create file simulation_results.csv.",
                     "Check write permissions in the current directory.",
                     "majboroda@example.com"}},
        {"ERR-002", {"System parameter error",
                     "Parameter T (time constant) equals zero.",
                     "Restart and enter T > 0 (recommended: 0.1).",
                     "majboroda@example.com"}},
        {"ERR-003", {"Simulation error",
                     "An unexpected error occurred during integration.",
                     "Restart the program. If the error persists, contact the developer.",
                     "majboroda@example.com"}},
        {"ERR-004", {"Visualization error",
                     "Failed to run Python script visualize.py.",
                     "Run manually: python3 visualize.py",
                     "majboroda@example.com"}},
    };
};