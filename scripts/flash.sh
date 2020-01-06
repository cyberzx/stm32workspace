cd "$(dirname "$0")"

PROGRAM=$1
OPENOCD_SCRIPTS=${OPENOCD_SCRIPTS:-/usr/share/openocd/scripts}

if [ -z "$PROGRAM" ]
then
  echo "Usage: $0 program-name"
  exit
fi

openocd -s ${OPENOCD_SCRIPTS} -f ../openocd/csk32f103.cfg  -c "program $PROGRAM verify 0x08000000" -c exit
st-flash reset
