from setuptools import setup, find_packages

setup(
    name='muri_logics',
    version='0.1.0',
    packages=find_packages(),      # findet automatisch alles im Ordner
    install_requires=[
        # hier deine Python-Abhängigkeiten eintragen, z.B. numpy, requests
    ],
    author='MURI Dev-Team',
    description='MURI Logic Python Package',
    python_requires='>=3.10',
)
