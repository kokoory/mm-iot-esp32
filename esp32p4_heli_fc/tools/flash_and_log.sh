#!/usr/bin/env bash
#
# flash_and_log.sh - ESP32-P4 Helicopter FC 빌드/플래시/로그 캡처 스크립트
#
# 사용법:
#   ./tools/flash_and_log.sh              # 빌드 + 플래시 + 모니터 (로그 저장)
#   ./tools/flash_and_log.sh build        # 빌드만
#   ./tools/flash_and_log.sh flash        # 플래시만 + 모니터
#   ./tools/flash_and_log.sh monitor      # 모니터만 (로그 저장)
#   ./tools/flash_and_log.sh clean        # 빌드 디렉토리 삭제 후 재빌드
#
# 로그 파일은 logs/ 디렉토리에 타임스탬프 기반으로 저장됩니다.
#   예: logs/2026-03-20_143025_flash.log
#
set -euo pipefail

# ── 프로젝트 디렉토리 (이 스크립트의 상위 디렉토리) ──
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
LOG_DIR="$PROJECT_DIR/logs"

# ── 색상 출력 ──
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

info()  { echo -e "${GREEN}[INFO]${NC} $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC} $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; }

# ── 로그 디렉토리 생성 ──
mkdir -p "$LOG_DIR"

# ── 타임스탬프 파일명 생성 ──
TIMESTAMP="$(date +%Y-%m-%d_%H%M%S)"

# ── ESP-IDF 환경 확인 ──
check_idf() {
    if ! command -v idf.py &>/dev/null; then
        error "idf.py를 찾을 수 없습니다."
        echo "  ESP-IDF 환경을 먼저 설정하세요:"
        echo "    source \$IDF_PATH/export.sh"
        echo "  또는:"
        echo "    . ~/esp/esp-idf/export.sh"
        exit 1
    fi
    info "ESP-IDF 환경 확인 완료: $(idf.py --version 2>/dev/null || echo 'version unknown')"
}

# ── 빌드 ──
do_build() {
    local log_file="$LOG_DIR/${TIMESTAMP}_build.log"
    info "빌드 시작... (로그: $log_file)"
    cd "$PROJECT_DIR"

    # tee로 콘솔 + 파일 동시 출력
    idf.py build 2>&1 | tee "$log_file"

    if [ "${PIPESTATUS[0]}" -eq 0 ]; then
        info "빌드 성공!"
    else
        error "빌드 실패. 로그를 확인하세요: $log_file"
        exit 1
    fi
}

# ── 플래시 ──
do_flash() {
    local log_file="$LOG_DIR/${TIMESTAMP}_flash.log"
    info "플래시 시작... (로그: $log_file)"
    cd "$PROJECT_DIR"

    idf.py flash 2>&1 | tee "$log_file"

    if [ "${PIPESTATUS[0]}" -eq 0 ]; then
        info "플래시 성공!"
    else
        error "플래시 실패. 로그를 확인하세요: $log_file"
        echo ""
        echo -e "${CYAN}문제 해결 팁:${NC}"
        echo "  1. USB 케이블이 연결되어 있는지 확인"
        echo "  2. 포트 권한 확인: sudo usermod -aG dialout \$USER"
        echo "  3. 수동 포트 지정: idf.py -p /dev/ttyUSB0 flash"
        echo "  4. ESP32-P4 보드의 BOOT 버튼을 누른 상태에서 RESET"
        exit 1
    fi
}

# ── 모니터 (로그 캡처) ──
do_monitor() {
    local log_file="$LOG_DIR/${TIMESTAMP}_monitor.log"
    info "시리얼 모니터 시작 (Ctrl+] 로 종료)"
    info "로그 저장 위치: $log_file"
    echo ""
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${CYAN} RPC 통신 확인 방법:${NC}"
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo "  1. [RPC STATS] 로그가 5초마다 출력됩니다"
    echo "     - telem sent/recv 숫자가 증가하면 Core0→Core1 정상"
    echo "     - cmd sent/recv 숫자가 증가하면 Core1→Core0 정상"
    echo "     - drop 숫자가 많으면 큐 오버플로우 (처리 속도 문제)"
    echo ""
    echo "  2. 'cmd RX' 로그: GCS에서 명령이 도착했음을 의미"
    echo "  3. 'telem TX' 로그: FC가 텔레메트리를 전송했음을 의미"
    echo ""
    echo "  4. HaLow 연결 확인:"
    echo "     - 'Link UP - IP:' 가 보이면 WiFi 연결 성공"
    echo "     - 'GCS bridge ready on UDP port 14550' 이 보이면 준비 완료"
    echo ""
    echo "  5. GCS(QGroundControl/Mission Planner) 연결:"
    echo "     - UDP 14550 포트로 자동 연결됨"
    echo "     - Heartbeat가 보이면 통신 성공"
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""
    cd "$PROJECT_DIR"

    # idf.py monitor의 출력을 파일과 화면 동시 출력
    idf.py monitor 2>&1 | tee "$log_file"

    info "모니터 종료. 로그 저장 완료: $log_file"
}

# ── 클린 빌드 ──
do_clean() {
    info "빌드 디렉토리 삭제 중..."
    cd "$PROJECT_DIR"
    idf.py fullclean
    info "클린 완료. 재빌드합니다..."
    do_build
}

# ── 오래된 로그 정리 (30일 이상) ──
cleanup_old_logs() {
    local count
    count=$(find "$LOG_DIR" -name "*.log" -mtime +30 2>/dev/null | wc -l)
    if [ "$count" -gt 0 ]; then
        warn "30일 이상 된 로그 $count 개를 삭제합니다..."
        find "$LOG_DIR" -name "*.log" -mtime +30 -delete
    fi
}

# ── 메인 ──
main() {
    echo -e "${CYAN}╔══════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║  ESP32-P4 Helicopter FC - Flash & Log Tool  ║${NC}"
    echo -e "${CYAN}╚══════════════════════════════════════════════╝${NC}"
    echo ""

    check_idf
    cleanup_old_logs

    local cmd="${1:-all}"

    case "$cmd" in
        build)
            do_build
            ;;
        flash)
            do_flash
            do_monitor
            ;;
        monitor)
            do_monitor
            ;;
        clean)
            do_clean
            ;;
        all|"")
            do_build
            do_flash
            do_monitor
            ;;
        *)
            echo "사용법: $0 {build|flash|monitor|clean|all}"
            echo ""
            echo "  build   - 빌드만 수행"
            echo "  flash   - 플래시 + 모니터"
            echo "  monitor - 모니터만 (로그 캡처)"
            echo "  clean   - 클린 빌드"
            echo "  all     - 빌드 + 플래시 + 모니터 (기본값)"
            exit 1
            ;;
    esac
}

main "$@"
