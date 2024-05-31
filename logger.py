class Logger:
    def __init__(self, filename):
        self.filename = filename
        self.logs = []

    def log(self, msg):
        self.logs.append(msg)

    def save(self):
        try:
            with open(self.filename, 'w') as f:
                for log in self.logs:
                    f.write(f"{log}\n")
        except Exception as e:
            print(f"Error al guardar el archivo: {e}")
