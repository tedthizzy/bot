# Future FastAPI service serving the face and teleop pages.
# Publish face expressions over WebSocket using roverlib message contracts.
# Forward teleop SkillCall requests to robotd with source=web.
# Reserve rate-limited POST /still and POST /cmd stubs with source=phone commands.
# Derive request validation and rover skill bounds from roverlib.
