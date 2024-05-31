#!/bin/bash

# Nombre del entorno virtual
ENV_NAME="myenv"

# Ruta al archivo de configuración de pip
PIP_CONFIG_FILE="$HOME/.pip/pip.conf"

# Renombrar el archivo de configuración temporalmente si existe
if [ -f "$PIP_CONFIG_FILE" ]; then
    mv "$PIP_CONFIG_FILE" "${PIP_CONFIG_FILE}.bak"
fi

# Crear el entorno virtual
virtualenv $ENV_NAME

# Activar el entorno virtual
source $ENV_NAME/bin/activate

# Función para instalar paquetes y verificar errores
install_package() {
    package=$1
    version=$2
    pip install --index-url https://pypi.org/simple "${package}==${version}"
    if [ $? -ne 0 ]; then
        echo "Error installing ${package}==${version}. Exiting."
        exit 1
    fi
}

# Instalar los paquetes necesarios con las versiones especificadas
install_package opencv-python 
install_package numpy 1.24.4
install_package ultralytics 8.0.230
install_package torch 2.3.0

# Restaurar el archivo de configuración de pip si fue renombrado
if [ -f "${PIP_CONFIG_FILE}.bak" ]; then
    mv "${PIP_CONFIG_FILE}.bak" "$PIP_CONFIG_FILE"
fi

echo "Environment setup complete. To activate the environment, use: source $ENV_NAME/bin/activate"
