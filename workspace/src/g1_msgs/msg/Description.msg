# A describer's answer to a DescribeRequest.
string subject_id
string task

# Canonical name (1-3 words) and one-sentence caption, for objects.
string name
string caption

# Whether the detector's label fits what the describer sees.
bool label_ok

# For rooms: one of the room types in the world model's table.
string room_type

float32 confidence

# Which backend answered, for logs only.
string model
