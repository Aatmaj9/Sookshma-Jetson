BAGS_DIR="./bags/images"
SCRIPT="./zed2i/capture_image.py"

mkdir -p "$BAGS_DIR"

# find the next test number
n=1
while [ -d "$BAGS_DIR/test$n" ]; do
    ((n++))
done

SAVE_DIR="$BAGS_DIR/test$n"
mkdir -p "$SAVE_DIR"

echo "🟢 Starting new capture session in: $SAVE_DIR"
echo "Press SPACE to save an image, Ctrl+C to stop."

python3 "$SCRIPT" "$SAVE_DIR"
