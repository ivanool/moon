#!/usr/bin/env python3
"""
Conversor de GIF a imágenes binarias comprimidas para ESP32 ST7789
- Extrae frames de un GIF
- Redimensiona a 240x135 píxeles (resolución real de pantalla)
- Convierte a formato RGB565
- Comprime para mantener bajo 1.8 MB total
- Guarda archivos .bin numerados

INSTRUCCIONES:
1. Coloca tu archivo GIF en el directorio tools/
2. Modifica la variable GIF_PATH con el nombre de tu archivo
3. Ejecuta: python image_compressed.py
4. Los archivos .bin se guardarán en spiffs_image/
"""

import os
import sys
from PIL import Image, ImageOps
import struct
import math

# ========================================
# CONFIGURACIÓN - MODIFICA ESTAS VARIABLES
# ========================================

# Configuración de pantalla ST7789 - Resolución real
TFT_WIDTH = 240  # Ancho real de la pantalla
TFT_HEIGHT = 135  # Alto real de la pantalla

# Configuración del archivo a procesar
GIF_PATH = "tools/makimareze.gif"  # Cambia este path por tu GIF
OUTPUT_DIR = "../spiffs_image/"

def rgb888_to_rgb565(r, g, b):
    """Convierte RGB888 a RGB565"""
    r = (r >> 3) & 0x1F
    g = (g >> 2) & 0x3F  
    b = (b >> 3) & 0x1F
    return (r << 11) | (g << 5) | b

def extract_gif_frames(gif_path, max_frames=None):
    """Extrae frames de un GIF"""
    try:
        gif = Image.open(gif_path)
        frames = []
        frame_count = 0
        
        while True:
            try:
                # Convertir a RGB si es necesario
                if gif.mode != 'RGB':
                    frame = gif.convert('RGB')
                else:
                    frame = gif.copy()
                
                frames.append(frame)
                frame_count += 1
                
                if max_frames and frame_count >= max_frames:
                    break
                    
                gif.seek(gif.tell() + 1)
                
            except EOFError:
                break
                
        print(f"Extraídos {len(frames)} frames del GIF")
        return frames
        
    except Exception as e:
        print(f"Error al abrir GIF: {e}")
        return []

def calculate_optimal_frames(gif_path, target_size_mb=1.8):
    """Calcula cuántos frames podemos extraer para no superar el tamaño objetivo"""
    target_size_bytes = target_size_mb * 1024 * 1024
    bytes_per_frame = TFT_WIDTH * TFT_HEIGHT * 2  # RGB565 = 2 bytes por pixel
    max_frames = int(target_size_bytes / bytes_per_frame)
    
    # Forzar 35 frames como se solicita
    desired_frames = 35
    
    # Verificar cuántos frames tiene el GIF
    try:
        gif = Image.open(gif_path)
        gif_frames = 0
        while True:
            try:
                gif.seek(gif_frames)
                gif_frames += 1
            except EOFError:
                break
        
        optimal_frames = min(desired_frames, gif_frames)
        print(f"GIF tiene {gif_frames} frames")
        print(f"Máximo posible para {target_size_mb}MB: {max_frames} frames")
        print(f"Frames solicitados: {desired_frames}")
        print(f"Frames a extraer: {optimal_frames}")
        print(f"Tamaño estimado: {(optimal_frames * bytes_per_frame) / (1024*1024):.2f} MB")
        
        return optimal_frames
        
    except Exception as e:
        print(f"Error al analizar GIF: {e}")
        return 35  # Valor por defecto cambiado a 35

def resize_and_optimize_frame(frame, target_width=TFT_WIDTH, target_height=TFT_HEIGHT):
    """Redimensiona y optimiza un frame"""
    # Redimensionar manteniendo proporción
    frame.thumbnail((target_width, target_height), Image.Resampling.LANCZOS)
    
    # Crear imagen con fondo negro del tamaño objetivo
    optimized = Image.new('RGB', (target_width, target_height), (0, 0, 0))
    
    # Centrar la imagen redimensionada
    x_offset = (target_width - frame.width) // 2
    y_offset = (target_height - frame.height) // 2
    optimized.paste(frame, (x_offset, y_offset))
    
    return optimized

def frame_to_rgb565_binary(frame):
    """Convierte un frame a formato binario RGB565"""
    binary_data = bytearray()
    
    for y in range(TFT_HEIGHT):
        for x in range(TFT_WIDTH):
            r, g, b = frame.getpixel((x, y))
            rgb565 = rgb888_to_rgb565(r, g, b)
            # Little endian para ESP32
            binary_data.extend(struct.pack('<H', rgb565))
    
    return binary_data

def save_frames_as_images(frames, output_dir):
    """Guarda frames como imágenes JPG para verificación"""
    os.makedirs(output_dir, exist_ok=True)
    
    for i, frame in enumerate(frames, 1):
        frame_path = os.path.join(output_dir, f"frame_{i:02d}.jpg")
        frame.save(frame_path, "JPEG", quality=90)
    
    print(f"Guardadas {len(frames)} imágenes de verificación en {output_dir}")

def process_gif_to_binary(gif_path, output_dir=".", save_images=True):
    """Procesa un GIF completo y genera archivos binarios"""
    if not os.path.exists(gif_path):
        print(f"Error: No se encuentra el archivo {gif_path}")
        return False
    
    print(f"Procesando GIF: {gif_path}")
    
    # Calcular frames óptimos
    optimal_frames = calculate_optimal_frames(gif_path)
    
    # Extraer frames
    frames = extract_gif_frames(gif_path, optimal_frames)
    if not frames:
        print("No se pudieron extraer frames del GIF")
        return False
    
    # Procesar cada frame
    processed_frames = []
    total_size = 0
    
    print("Procesando frames...")
    for i, frame in enumerate(frames, 1):
        print(f"Procesando frame {i}/{len(frames)}")
        
        # Redimensionar y optimizar
        optimized_frame = resize_and_optimize_frame(frame)
        processed_frames.append(optimized_frame)
        
        # Convertir a binario
        binary_data = frame_to_rgb565_binary(optimized_frame)
        
        # Guardar archivo binario
        bin_filename = f"{i}.bin"
        bin_path = os.path.join(output_dir, bin_filename)
        
        with open(bin_path, 'wb') as f:
            f.write(binary_data)
        
        total_size += len(binary_data)
        print(f"Guardado: {bin_filename} ({len(binary_data):,} bytes)")
    
    # Guardar imágenes de verificación si se solicita
    if save_images and processed_frames:
        images_dir = os.path.join(output_dir, "verification_images")
        save_frames_as_images(processed_frames, images_dir)
    
    # Estadísticas finales
    total_mb = total_size / (1024 * 1024)
    print(f"\n=== RESUMEN ===")
    print(f"Frames procesados: {len(processed_frames)}")
    print(f"Tamaño total: {total_size:,} bytes ({total_mb:.2f} MB)")
    print(f"Promedio por frame: {total_size // len(processed_frames):,} bytes")
    print(f"Archivos generados en: {output_dir}")
    
    if total_mb <= 1.8:
        print("✅ Tamaño dentro del límite de 1.8 MB")
    else:
        print("⚠️  Tamaño excede 1.8 MB - considera reducir frames")
    
    return True

def main():
    # Usar configuración definida en el código
    gif_path = GIF_PATH
    output_dir = OUTPUT_DIR
    
    # Verificar que el archivo GIF existe
    if not os.path.exists(gif_path):
        print(f"Error: No se encuentra el archivo {gif_path}")
        print("Por favor, modifica la variable GIF_PATH en el código")
        sys.exit(1)
    
    print(f"Configuración:")
    print(f"- Resolución pantalla: {TFT_WIDTH}x{TFT_HEIGHT}")
    print(f"- Archivo GIF: {gif_path}")
    print(f"- Directorio salida: {output_dir}")
    print()
    
    # Crear directorio de salida si no existe
    os.makedirs(output_dir, exist_ok=True)
    
    success = process_gif_to_binary(gif_path, output_dir)
    
    if success:
        print("\n🎉 Conversión completada exitosamente!")
        print(f"Los archivos .bin están listos para usar en ESP32")
    else:
        print("\n❌ Error en la conversión")
        sys.exit(1)

if __name__ == "__main__":
    main()
