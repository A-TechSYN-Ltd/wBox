#!/bin/bash

echo "GNSS Docker sistemi güncelleniyor..."

# GNSS logger için repo adresi
REPO_URL="https://github.com/EnesTayfun/wBox-v2.git"
LOCAL_DIR="/home/wBox"

# Eger klasör yoksa klonla, varsa pull yap
if [ ! -d "$LOCAL_DIR" ]; then
  echo "Klasör bulunamadi. Repo klonlaniyor..."
  git clone "$REPO_URL" "$LOCAL_DIR"
else
  echo "?? Klasör bulundu. Güncellemeler çekiliyor..."
  cd "$LOCAL_DIR" || exit
  git pull
fi

# Docker islemleri
cd "$LOCAL_DIR" || exit

echo "Docker image build ediliyor..."
docker-compose build

echo "Container baslatiliyor..."
docker-compose up -d

echo "Güncelleme tamamlandi!"
