#!/bin/bash
# setup-github-cli.sh - Скрипт для установки и настройки GitHub CLI в CI среде

set -e  # Прервать выполнение при ошибке

echo "=== Установка и настройка GitHub CLI ==="

# Проверка, установлен ли уже GitHub CLI
if command -v gh &> /dev/null; then
    echo "GitHub CLI уже установлен: $(gh --version)"
else
    echo "Установка GitHub CLI..."

    # Установка GitHub CLI в зависимости от операционной системы
    if [ "$(uname)" = "Linux" ]; then
        # Для Ubuntu/Debian
        if command -v apt-get &> /dev/null; then
            curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg | sudo dd of=/usr/share/keyrings/githubcli-archive-keyring.gpg
            echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | sudo tee /etc/apt/sources.list.d/github-cli.list > /dev/null
            sudo apt update
            sudo apt install gh -y
        # Для CentOS/RHEL/Fedora
        elif command -v yum &> /dev/null || command -v dnf &> /dev/null; then
            sudo yum install gh -y || sudo dnf install gh -y
        else
            echo "Не удалось определить пакетный менеджер. Установите GitHub CLI вручную."
            exit 1
        fi
    elif [ "$(uname)" = "Darwin" ]; then
        # Для macOS
        if command -v brew &> /dev/null; then
            brew install gh
        else
            echo "Установите Homebrew или GitHub CLI вручную для macOS."
            exit 1
        fi
    else
        echo "Неподдерживаемая операционная система: $(uname)"
        exit 1
    fi
fi

# Проверка версии GitHub CLI
echo "GitHub CLI версия: $(gh --version)"

# Настройка GitHub CLI
echo "Настройка GitHub CLI..."
gh config set pager cat
gh config set editor nano
gh config set git_protocol https

# Проверка аутентификации
if gh auth status 2>/dev/null; then
    echo "GitHub CLI уже аутентифицирован"
    echo "Текущий пользователь: $(gh api user --jq '.login')"
else
    echo "GitHub CLI не аутентифицирован"
    if [ -n "$GITHUB_TOKEN" ]; then
        echo "Используем предоставленный GITHUB_TOKEN для аутентификации"
        echo "$GITHUB_TOKEN" | gh auth login --with-token
        echo "Аутентификация прошла успешно"
        echo "Текущий пользователь: $(gh api user --jq '.login')"
    else
        echo "GITHUB_TOKEN не установлен. Необходимо настроить аутентификацию вручную."
        exit 1
    fi
fi

echo "=== Установка и настройка GitHub CLI завершена ==="

# Вывод справочной информации
echo ""
echo "=== Справка по использованию GitHub CLI ==="
echo "Примеры команд:"
echo "  gh repo view --json nameWithOwner,description"
echo "  gh issue list --state open"
echo "  gh pr list --state open"
echo "  gh api user --jq '.login'"
echo "  gh auth status"
echo ""

# Проверка наличия необходимых разрешений
echo "=== Проверка разрешений ==="
if [ -n "$GITHUB_TOKEN" ]; then
    # Получение информации о токене
    TOKEN_INFO=$(gh api user --jq '.login,.email,.name' 2>/dev/null || echo "ошибка получения информации")
    echo "Токен позволяет получить информацию о пользователе: $TOKEN_INFO"
fi

echo "=== Готово ==="