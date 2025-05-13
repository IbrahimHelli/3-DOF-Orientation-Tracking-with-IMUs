import processing.serial.*;
Serial myPort;

float qw, qx, qy, qz;
float rotX, rotY, rotZ;

void setup() {
    size(600, 600, P3D);
    myPort = new Serial(this, "COM5", 115200);
    myPort.bufferUntil('\n');
}

void draw() {
    background(0);
    lights();
    translate(width / 2, height / 2, 0);

    applyQuaternionRotation();

    // X ekseni (Kırmızı) → Sağa gidiyor
    stroke(255, 0, 0);
    line(0, 0, 0, 150, 0, 0);
    fill(255, 0, 0);
    text("X", 160, 0, 0);

    // Y ekseni (Yeşil) → Sayfanın içine gidiyor
    stroke(0, 255, 0);
    line(0, 0, 0, 0, 0, -150);
    fill(0, 255, 0);
    text("Y", 0, 0, -160); // Y ekseni etiketi
    
    // Z ekseni (Mavi) → Yukarı gidiyor
    stroke(0, 0, 255);
    line(0, 0, 0, 0, -150, 0);
    fill(0, 0, 255);
    text("Z", 0, -160, 0); // Z ekseni etiketi

    fill(255, 255);
    box(100);
}

// Kuaterniyonları Euler açılara çevirme
void applyQuaternionRotation() {
    float sinr_cosp = 2 * (qw * qx + qy * qz);
    float cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    rotX = atan2(sinr_cosp, cosr_cosp);

    float sinp = 2 * (qw * qy - qz * qx);
    if (abs(sinp) >= 1)
        rotY = (sinp >= 0 ? HALF_PI : -HALF_PI);
    else
        rotY = asin(sinp);

    float siny_cosp = 2 * (qw * qz + qx * qy);
    float cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    rotZ = atan2(siny_cosp, cosy_cosp);

    rotateX(-rotX);
    rotateY(rotZ); // Z Y ekseni olarak hareket etmeli
    rotateZ(rotY);
}

// Seri porttan veri okuma
void serialEvent(Serial p) {
    String data = p.readStringUntil('\n');
    if (data != null) {
        data = trim(data);
        String[] parts = split(data, ",");
        if (parts.length == 4) {
            qw = float(parts[0]);
            qx = float(parts[1]);
            qy = float(parts[2]);
            qz = float(parts[3]);
        }
    }
}
