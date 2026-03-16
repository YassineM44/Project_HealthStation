# Project_HealthStation

Health Station is a full-stack embedded health monitoring system built around an ESP32 custom PCB and a Raspberry Pi Zero 2W. It acquires biometric data — ECG (AD8232), heart rate & SpO₂ (MAX30102), body temperature (MLX90614), and pill intake via a load cell + HX711 — processes it locally with anomaly detection, and syncs to a cloud backend over HTTPS/TLS. A mobile dashboard provides live vitals, historical trends, threshold configuration, and pill schedule management. Designed for educational purposes only — not a certified medical device.
