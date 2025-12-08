# Compiling Controllers

## For macOS Users

Use the provided script:
```bash
./compile_from_terminal.sh
```

## For Windows Users

Windows users have two options:

### Option 1: Compile from Webots GUI (Recommended)
1. Open the project in Webots
2. In Scene Tree, right-click on controller folder → **"Make"**
3. Repeat for each controller

This works the same on Windows and macOS!

### Option 2: Compile from Terminal

According to Webots documentation, on Windows:

1. Open Command Prompt or PowerShell
2. Set WEBOTS_HOME environment variable:
   ```cmd
   set WEBOTS_HOME=C:\Program Files\Webots
   ```
   (Adjust path to your Webots installation)

3. Navigate to controller directory and compile:
   ```cmd
   cd controllers\rosbot
   make clean
   make
   ```

Webots on Windows includes MinGW compiler, so no additional setup needed.

## For Linux Users

Same as Windows terminal method, but use:
```bash
export WEBOTS_HOME=/usr/local/webots
cd controllers/rosbot
make clean && make
```

---

**Note:** The Windows `.exe` files were removed to allow cross-platform compilation. All source files (`.c`) and Makefiles remain and work on all platforms.





