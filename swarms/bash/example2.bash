#!/bin/bash

# Extract network configurations
network_configs=$(cat <<EOF
network={
      ssid="open"
      key_mgmt=NONE
      id_str="open"
      priority=3
}
# Add other network configurations here
EOF
)

# Function to extract the value of a given field from a network config block
extract_field() {
  local block="\$1"
  local field="\$2"
  echo "$block" | grep -E "^\\s*$field=" | cut -d '=' -f 2 | tr -d '"'
}

# Function to check internet connectivity
is_online() {
  local ssid="\$1"
  # You can use wget, curl, or ping to check connectivity
  # This example uses ping to check connectivity to 8.8.8.8 (Google DNS)
  ping -q -w 1 -c 1 8.8.8.8 > /dev/null 2>&1
  return $?
}

highest_priority=-1
highest_priority_ssid=""

# Process each network config block
while read -r block; do
  ssid=$(extract_field "$block" "ssid")
  priority=$(extract_field "$block" "priority")

  if is_online "$ssid"; then
    if [ "$priority" -gt "$highest_priority" ]; then
      highest_priority="$priority"
      highest_priority_ssid="$ssid"
    fi
  fi
done < <(echo "$network_configs" | awk '/^network=/ { buf = \$0; while (getline > 0) { buf = buf "\n" \$0; if (\$0 ~ /^}/) { print buf; buf = "" } } }')

if [ -z "$highest_priority_ssid" ]; then
  echo "No available network with the highest priority"
else
  echo "Highest priority connected network: $highest_priority_ssid (priority: $highest_priority)"
fi