# Touchscreen Setup

This package uses a regular GTK application for the touchscreen panel.  Touch
input is handled by the Linux desktop input stack, not by `dvrk_display`
directly.

## 1. Confirm the Session Type

These instructions use `xinput` and `xrandr`, so they apply to an X11 session.

```sh
echo "$XDG_SESSION_TYPE"
```

Expected output:

```text
x11
```

If the session is `wayland`, either switch the user to an Xorg/X11 session or
configure the touchscreen through the desktop environment's Wayland settings.

## 2. Find the Touch Device

After plugging in the monitor's USB cable, list the input devices:

```sh
xinput list
```

Look for the touchscreen controller.  On the current system this is:

```text
pointer:TSTP MTouch
```

If the device is not listed, the OS does not see the touch controller yet.
Check the monitor's USB upstream cable, USB hub, and any required monitor
settings.

## 3. Find the Video Output

List the active displays:

```sh
xrandr --listmonitors
```

The output name is the connector name used by XRandR, for example `DP-2`.

To test the current setup, map the touch device to the display:

```sh
xinput map-to-output "pointer:TSTP MTouch" DP-2
```

If touch now lands on the correct monitor, the input device and output name are
correct.

## 4. Identify the Monitor Model

Hard-coding `DP-2` works only while the monitor stays on that same video port.
For a shared system, identify the monitor by its EDID model instead.

GNOME records display names in the user's monitor configuration:

```sh
grep -A5 -B2 -E "CEX|CX156|connector|product|serial" ~/.config/monitors.xml
```

On the current system, GNOME shows the touchscreen monitor as:

```text
connector: DP-2
vendor:    CEX
product:   CX156
serial:    0x00000001
```

You can also inspect EDID information directly if `edid-decode` is installed:

```sh
for edid in /sys/class/drm/card*-*/edid; do
    [ -s "$edid" ] || continue
    output=${edid%/edid}
    output=${output##*/}
    echo "$output"
    edid-decode "$edid" 2>/dev/null | grep -E "Manufacturer|Model|Display Product Name|Display Product Serial"
done
```

Note that `/sys/class/drm` names and XRandR names do not always match exactly.
Use XRandR output names, such as `DP-2`, with `xinput map-to-output`.

## 5. Make a Fixed-Port Setup Persistent

If the monitor will always be connected to the same output, create a small
helper script and let the desktop start it on login.  The helper logs every run
so it is easy to tell whether the desktop actually launched it:

```sh
sudo tee /usr/local/bin/dvrk-map-touchscreen >/dev/null <<'EOF'
#!/bin/sh

LOG_FILE="/tmp/dvrk-touchscreen-map-${USER:-unknown}.log"
exec >>"$LOG_FILE" 2>&1

echo "==== $(date) ===="
echo "DISPLAY=${DISPLAY:-} XDG_SESSION_TYPE=${XDG_SESSION_TYPE:-}"

for attempt in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20; do
    if xinput list "pointer:TSTP MTouch" >/dev/null 2>&1 \
       && xrandr --query | grep -q "^DP-2 connected"; then
        echo "Mapping pointer:TSTP MTouch to DP-2"
        xinput map-to-output "pointer:TSTP MTouch" DP-2
        exit $?
    fi
    echo "Attempt $attempt: touch device or DP-2 not ready"
    sleep 1
done

echo "Unable to map touchscreen"
exit 1
EOF

sudo chmod 755 /usr/local/bin/dvrk-map-touchscreen
```

For GNOME, use XDG autostart:

```sh
sudo tee /etc/xdg/autostart/dvrk-touchscreen.desktop >/dev/null <<'EOF'
[Desktop Entry]
Type=Application
Name=dVRK Touchscreen Mapping
Exec=/usr/local/bin/dvrk-map-touchscreen
NoDisplay=true
X-GNOME-Autostart-enabled=true
EOF
```

If XDG autostart does not run on your system, use this `/etc/X11/Xsession.d`
fallback instead.  These files are sourced by the session and normally use mode
`644`; they do not need executable permissions:

```sh
sudo tee /etc/X11/Xsession.d/90-dvrk-touchscreen >/dev/null <<'EOF'
/usr/local/bin/dvrk-map-touchscreen &
EOF

sudo chmod 644 /etc/X11/Xsession.d/90-dvrk-touchscreen
```

Log out and back in, then test the touchscreen again.

## 6. Make a Model-Based Setup Persistent

Use this version if the monitor may be plugged into a different video port.  It
looks for the XRandR output whose EDID contains the touchscreen monitor product
name, then maps the touch device to that output.

Install the helper tools if needed:

```sh
sudo apt install xinput x11-xserver-utils edid-decode xxd
```

Create a helper script:

```sh
sudo tee /usr/local/bin/dvrk-map-touchscreen >/dev/null <<'EOF'
#!/bin/sh

TOUCH_DEVICE_MATCH="TSTP MTouch"
MONITOR_PRODUCT="CX156"
LOG_FILE="/tmp/dvrk-touchscreen-map-${USER:-unknown}.log"

exec >>"$LOG_FILE" 2>&1

echo "==== $(date) ===="
echo "DISPLAY=${DISPLAY:-} XDG_SESSION_TYPE=${XDG_SESSION_TYPE:-}"
echo "Looking for touch device matching '$TOUCH_DEVICE_MATCH'"
echo "Looking for monitor product '$MONITOR_PRODUCT'"

find_touch_device() {
    xinput list --name-only | grep -F "$TOUCH_DEVICE_MATCH" | head -n 1
}

find_monitor_output() {
    tmp_dir=$(mktemp -d) || return 1

    xrandr --verbose | awk -v dir="$tmp_dir" '
        /^[^[:space:]]+ connected/ {
            output = $1
            edid_file = dir "/" output ".edid.hex"
            collecting = 0
            next
        }
        /^[^[:space:]]+ disconnected/ {
            output = ""
            collecting = 0
            next
        }
        output != "" && /^[[:space:]]+EDID:/ {
            collecting = 1
            next
        }
        collecting && /^[[:space:]]+[0-9a-fA-F]+$/ {
            gsub(/[[:space:]]/, "", $0)
            print >> edid_file
            next
        }
        collecting {
            collecting = 0
        }
    '

    for hex_file in "$tmp_dir"/*.edid.hex; do
        [ -s "$hex_file" ] || continue

        output=$(basename "$hex_file" .edid.hex)
        decoded=$(xxd -r -p "$hex_file" | edid-decode - 2>/dev/null)

        if printf '%s\n' "$decoded" | grep -Fqi "$MONITOR_PRODUCT"; then
            printf '%s\n' "$output"
            rm -rf "$tmp_dir"
            return 0
        fi
    done

    rm -rf "$tmp_dir"
    return 1
}

for attempt in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30; do
    touch_device=$(find_touch_device)
    monitor_output=$(find_monitor_output)
    echo "Attempt $attempt: touch_device='$touch_device' monitor_output='$monitor_output'"

    if [ -n "$touch_device" ] && [ -n "$monitor_output" ]; then
        echo "Mapping '$touch_device' to '$monitor_output'"
        xinput map-to-output "$touch_device" "$monitor_output"
        exit $?
    fi

    sleep 1
done

echo "Unable to map touchscreen"
exit 1
EOF

sudo chmod 755 /usr/local/bin/dvrk-map-touchscreen
```

Call it for every X11 login:

```sh
sudo tee /etc/xdg/autostart/dvrk-touchscreen.desktop >/dev/null <<'EOF'
[Desktop Entry]
Type=Application
Name=dVRK Touchscreen Mapping
Exec=/usr/local/bin/dvrk-map-touchscreen
NoDisplay=true
X-GNOME-Autostart-enabled=true
EOF
```

For systems that do not use XDG autostart, call the helper from `Xsession.d`
as a fallback.  These files are sourced by the session and normally use mode
`644`; they do not need executable permissions:

```sh
sudo tee /etc/X11/Xsession.d/90-dvrk-touchscreen >/dev/null <<'EOF'
/usr/local/bin/dvrk-map-touchscreen &
EOF

sudo chmod 644 /etc/X11/Xsession.d/90-dvrk-touchscreen
```

Log out and back in, then test the touchscreen again.

## 7. Disable the GNOME Virtual Keyboard

GNOME can show its on-screen keyboard when a touchscreen is present and a text
field gets focus.  For the dVRK touchscreen panel this is usually distracting.

For the current user, disable it in the desktop UI:

```text
Settings -> Accessibility -> Typing -> Screen Keyboard: Off
```

Or run:

```sh
gsettings set org.gnome.desktop.a11y.applications screen-keyboard-enabled false
```

For all users, set a dconf default:

```sh
sudo mkdir -p /etc/dconf/profile

if [ ! -f /etc/dconf/profile/user ]; then
    sudo tee /etc/dconf/profile/user >/dev/null <<'EOF'
user-db:user
system-db:local
EOF
elif ! grep -q '^system-db:local$' /etc/dconf/profile/user; then
    echo 'system-db:local' | sudo tee -a /etc/dconf/profile/user >/dev/null
fi

sudo mkdir -p /etc/dconf/db/local.d

sudo tee /etc/dconf/db/local.d/00-dvrk-disable-screen-keyboard >/dev/null <<'EOF'
[org/gnome/desktop/a11y/applications]
screen-keyboard-enabled=false
EOF

sudo dconf update
```

To prevent users from turning it back on accidentally, also add a dconf lock:

```sh
sudo mkdir -p /etc/dconf/db/local.d/locks

sudo tee /etc/dconf/db/local.d/locks/00-dvrk-disable-screen-keyboard >/dev/null <<'EOF'
/org/gnome/desktop/a11y/applications/screen-keyboard-enabled
EOF

sudo dconf update
```

Log out and back in after changing dconf settings.

If the keyboard still appears for this application, bypass IBus for the
control panel process:

```sh
GTK_IM_MODULE=gtk-im-context-simple ros2 run dvrk_console control_panel -c /path/to/touchscreen.json
```

Current versions of the control panel executable set this variable internally
because the app has no text-entry widgets.

## 8. Troubleshooting

Show input devices:

```sh
xinput list
```

Show active displays:

```sh
xrandr --listmonitors
```

Show detailed display properties and EDID blocks:

```sh
xrandr --verbose
```

Show the persistent mapper log:

```sh
cat "/tmp/dvrk-touchscreen-map-${USER}.log"
```

Check what the helper script detects:

```sh
/usr/local/bin/dvrk-map-touchscreen
echo $?
```

If the touch device is visible but touches are offset or land on the wrong
screen, rerun the mapping after the desktop has loaded:

```sh
xinput map-to-output "pointer:TSTP MTouch" DP-2
```

If that fixes the problem, update the persistent script's monitor product or
fixed output name.  Also make sure the persistent script runs after GNOME
applies the monitor layout.  A symptom of an early mapping is that touching the
screen moves or hides the mouse pointer, but taps do not land on the expected
application buttons.

Check whether GNOME's virtual keyboard is enabled:

```sh
gsettings get org.gnome.desktop.a11y.applications screen-keyboard-enabled
```
