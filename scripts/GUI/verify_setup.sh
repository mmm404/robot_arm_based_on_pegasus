#!/bin/bash
# Verification script for Pegasus Arm setup

echo "========================================"
echo "Pegasus Arm - Code Callability Check"
echo "========================================"

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

errors=0

echo ""
echo "Checking Python files..."

if [ -f ~/ros2-ws/scripts/GUI/GUI_commander.py ]; then
    echo -e "${GREEN}✓${NC} GUI_commander.py exists"
    python3 -m py_compile ~/ros2-ws/scripts/GUI/GUI_commander.py 2>/dev/null && \
        echo -e "${GREEN}✓${NC} GUI_commander.py syntax OK" || \
        { echo -e "${RED}✗${NC} GUI_commander.py syntax error"; ((errors++)); }
else
    echo -e "${RED}✗${NC} GUI_commander.py not found"; ((errors++))
fi

if [ -f ~/ros2-ws/src/update_pegasus_description/scripts/pegasus_commander.py ]; then
    echo -e "${GREEN}✓${NC} pegasus_commander.py exists"
    python3 -m py_compile ~/ros2-ws/src/update_pegasus_description/scripts/pegasus_commander.py 2>/dev/null && \
        echo -e "${GREEN}✓${NC} pegasus_commander.py syntax OK" || \
        { echo -e "${RED}✗${NC} pegasus_commander.py syntax error"; ((errors++)); }
else
    echo -e "${RED}✗${NC} pegasus_commander.py not found"; ((errors++))
fi

if [ -f ~/ros2-ws/scripts/run_full_stack.sh ]; then
    echo -e "${GREEN}✓${NC} run_full_stack.sh exists"
    bash -n ~/ros2-ws/scripts/run_full_stack.sh 2>/dev/null && \
        echo -e "${GREEN}✓${NC} run_full_stack.sh syntax OK" || \
        { echo -e "${RED}✗${NC} run_full_stack.sh syntax error"; ((errors++)); }
else
    echo -e "${RED}✗${NC} run_full_stack.sh not found"; ((errors++))
fi

echo ""
echo "Checking ROS2 environment..."

[ -f ~/ros2-ws/install/setup.bash ] && \
    echo -e "${GREEN}✓${NC} ROS2 install space found" || \
    { echo -e "${RED}✗${NC} ROS2 install space not found"; ((errors++)); }

echo ""
echo "========================================"
[ $errors -eq 0 ] && echo -e "${GREEN}✓ All checks passed!${NC}" || echo -e "${RED}✗ $errors error(s)${NC}"
exit $errors
