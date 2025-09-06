# CoMonitorEvent.msg

# --- Event type (use as enum) ---
uint8 TYPE_CORNER=0
uint8 TYPE_TRIGGER=1
uint8 TYPE_FIT=2
uint8 type

# --- Corner specific ---
string pos
# --- Fit specific ---
float32  r2