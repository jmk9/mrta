string task_id
string type                         # "PICK" | "DROP" | "CHARGE"
geometry_msgs/PoseStamped pose      # PICK/DROP/CHARGE 접근 포즈(예: pre-dock)
float32 payload_kg
builtin_interfaces/Time deadline