> If you keep your repo flatter (all `.ino` files directly under `software/`), just ignore the `uno-r4/` + `mega2560/` folders—the explanations below still apply.

---

# Hardware and circuits

This robot has **three fundamental subsystems**:

1. **Audio detection** (microphone + amplifier + ADC signal conditioning)
2. **Tuning pegs** (motors + motor drivers + mechanical peg adapters)
3. **Strumming arm** (motorized slider + driver + pick attachment)

Depending on what you’re trying to build, you might only need _one_ of these.

---

## 1) If you only want pitch detection (single string OR polyphonic)

You only need the **audio detection circuit**.

### What I used

- **Korg CM200BK clip-on contact microphone** (chosen because I wasn’t using an electric guitar, and I wanted vibration-based input instead of ambient audio)
- A breadboard **audio input / preamp circuit** based on Amanda Ghassaei’s guide:
  - [Arduino Audio Input (Amanda Ghassaei) — Instructables](https://www.instructables.com/Arduino-Audio-Input/)

I got inspired to use the Korg contact mic from Antonio Hinojosa Cabrera’s work:

- [“Automatic guitar tuner” — Antonio Hinojosa Cabrera (UPC thesis)](https://upcommons.upc.edu/entities/publication/d1f17706-ed6d-4329-bab4-19746a667733)

### The tiny change that fixed weeks of pain (the “signal fades over time” bug)

After I got the circuit running, I noticed a weird phenomenon:

> The signal gets picked up perfectly… until it doesn’t.  
> The amplitude would slowly fade and collapse toward the ADC midpoint.

The fix ended up being **extremely simple**:

✅ Add a **1 MΩ resistor** that provides a DC path (this stabilized the piezo contact mic behavior).

I explain the full story (and why it happens) in Week 3 of my blog:

- [Week 3 — Algorithmic Pivots, Torque Troubles, and a Breakthrough](https://dtsf.sites.gettysburg.edu/dtsf2025/uncategorized/week-3-algorithmic-pivots-torque-troubles-and-a-breakthrough/)

> Practical tip: If you’re using a contact mic / piezo source and your signal “mysteriously dies,” assume you need to think about bias currents + DC paths before rewriting your entire FFT code.

---

## 2) If you want an automatic strummer

If you want the robot to strum consistently (so your pitch detection is repeatable), you’ll need:

### Core parts

- A **NEMA 17 stepper motor** (strumming doesn’t require huge torque)
- A **stepper driver** (I used an **A4988** in my build)
- A **sliding mechanism** that moves a pick across the strings
  - I personally **dismantled an old 3D printer** in the lab for a linear slide, but any stable slider works (rails, drawer slides, belt + carriage, etc.)
- A **pick attachment**
  - I designed a pick mount that worked for my geometry—feel free to copy/modify it from `hardware/3D_sketches/`

### A tutorial I genuinely recommend

Rachel De Barros’ stepper tutorial helped a lot for wiring + driver setup:

- [Rachel De Barros — Stepper Motor Tutorial (YouTube)](https://www.youtube.com/watch?v=wcLeXXATCR4)

### Two strumming lessons I learned the hard way

- **Pick stiffness matters.** A harsh “attack” creates extra noise that hurts frequency detection.
  - My 3D-printed pick was too aggressive.
  - Switching to a small piece of **cardboard** produced a cleaner strum and much better FFT stability.
- **Strummer height matters.** Small height adjustments changed how clean the strum was, which changed FFT accuracy.

---

## 3) The most important part: tuning motors (and torque reality)

This is where your design choices diverge the most.

### Stepper vs DC (no “better,” only tradeoffs)

**Stepper motors**

- ✅ Easier precision control (you command steps)
- ✅ Nice for small instruments / lower torque requirements (often electric guitars)
- ❌ Can be bulky/expensive for high torque needs
- ❌ Torque drops significantly when moving (dynamic torque < holding torque)

**DC gear motors**

- ✅ High torque for cheap (especially with gearboxes)
- ✅ Often more compact _for the torque you get_
- ❌ Less precise unless you add feedback or carefully tune control
- ❌ Requires more care to avoid overshoot / “too much turning”

### The under-documented truth: torque is everything

Torque requirements vary by instrument, string gauge, tuners, and tension.

A useful ballpark I learned _specifically from acoustic guitar tuning under load_:

- **~0.3 N·m** to reliably move the **heaviest string** on the acoustic guitar I tested.

The “how we discovered it” story lives across my DTSF blog posts:

- [All weekly posts (project page)](https://dtsf.sites.gettysburg.edu/dtsf2025/taha)

### Static vs dynamic torque (why my NEMA 17 failed anyway)

A classic trap: spec sheets show **holding torque**, not what you get while moving.

Example from my build:

- My NEMA 17 stepper was rated around **0.4 N·m holding torque**
- It still failed on tuning pegs that needed only ~**0.3 N·m**
- Why? Because torque drops when the motor spins (dynamic torque can be _much_ lower)

A good starting reference on static torque context:

- [Static torque (ScienceDirect Topics)](https://www.sciencedirect.com/topics/engineering/static-torque)

### Choosing motors based on what you’re building

#### If you’re building a **single-string tuner**

Motor size doesn’t constrain you mechanically as much, so you can choose based on simplicity.

- I’d recommend **a stepper** for your first build because control is straightforward.
- I tried NEMA sizes up through **NEMA 17** (no gearbox, typical 3D-printer steppers).
  - None worked reliably for the highest-torque peg on my acoustic.

If you want to see a higher-torque stepper approach in action, this NEMA 23 project is a nice reference point:

- [MECH 5840 (Auburn) — Automatic Guitar Tuner (YouTube)](https://www.youtube.com/watch?v=DU2-K5PHwyM&t=147s)

For wiring, use:

- `hardware/system_wiring/` (photos/diagrams)
- plus Rachel De Barros’ stepper driver tutorial above.

#### If you want the **full package** (strum + polyphonic detect + tune multiple strings)

This is what my project focused on—and for my acoustic guitar setup, **DC gear motors were the move**.

The motors I used:

- **Greartisan DC 12V 20RPM gear motors** (37mm gearbox, centric output shaft)

Why they worked well:

- Excellent torque
- Fit constraints mattered a lot—**motor diameter was a real limiting factor** on the headstock geometry I used.
  - Anything much larger than ~37mm would have made multi-motor packing difficult.

To mount multiple motors:

- I designed a **3-motor box** and peg attachments:
  - See `hardware/3D_sketches/` (e.g., motor box + peg attachment designs)
- I used **L298N** motor drivers to run the DC motors (and later used a power architecture trick with their 5V regulator output).

---

# Software

All sketches are in `software/`. The goal here is to make it obvious **what to run for what build**.

## `six_string_FFT.ino`

**Use this if:**

- You want **pitch detection only**
- You want to experiment with **single-string or polyphonic detection**
- You’re still perfecting your microphone + amplifier signal quality

**What it does:**

- Reads analog audio
- Runs FFT
- Splits the FFT output into **string frequency windows** (centered around expected open-string fundamentals)
- Reports detected peak frequencies (and/or confidence metrics depending on your implementation)

> Note: In my build, FFT resolution mattered a lot. On the Arduino Mega, memory constraints pushed smaller FFT sizes, which affected detection quality compared to the Uno R4.

## `single_string_stepper.ino`

**Use this if:**

- You’re building a **single-string automatic tuner**
- You’re using a **stepper motor** to turn a peg
- You want a clean first “closed loop” tuning system

**What it does:**

- Detects a target string frequency
- Computes error vs target
- Uses a PID-style response to decide how to turn
- Commands a stepper driver (example setup: A4988)

## `single_string_DC.ino`

**Use this if:**

- You’re tuning **one string** but using a **DC gear motor**
- You want to validate torque + control behavior before going multi-string

**What it does:**

- Detects pitch
- Computes error vs target
- Uses PID logic (or a simplified control law)
- Commands a DC motor via an H-bridge driver (example: **L298N**)
- Typically includes speed limiting / incremental turns to avoid overshoot

## `Three_Strings_Final_Demo.ino`

**Use this if:**

- You want the integrated system:
  - **Automatic strum**
  - **Multi-string pitch detection**
  - **Automatic tuning of multiple pegs** (my demo module was 3 strings)

**What it does (high level loop):**

1. Strum back and forth
2. Wait briefly for attack noise to settle
3. Collect FFT samples for a short window
4. Compute robust pitch estimates (I used **median frequency** per string band)
5. Compute tuning corrections (PID response)
6. Drive the correct motors

This sketch is the closest to the “full robot behavior.”

---

# Acknowledgements

A random video from **Tim Bell**—building an automatic guitar tuner more than a decade ago—flipped a switch in my head. It made the project feel _real_, like “wait… this is absolutely buildable.”

- [Tim Bell — Automatic Guitar Tuner (YouTube)](https://www.youtube.com/watch?v=ZahpROn3gm0)

None of this would have been possible without the support of:

- **Professor Eric Remy**
- **Joshua Wagner**

Thank you for the guidance, patience, and for helping me push through the very real “hardware meets reality” moments.

---

# References

Audio input + circuit inspiration:

- Amanda Ghassaei, _Arduino Audio Input_ (Instructables)  
  https://www.instructables.com/Arduino-Audio-Input/
- Antonio Hinojosa Cabrera, _Automatic guitar tuner_ (UPC thesis)  
  https://upcommons.upc.edu/entities/publication/d1f17706-ed6d-4329-bab4-19746a667733
- Week 3 blog post (piezo fade fix with 1 MΩ resistor)  
  https://dtsf.sites.gettysburg.edu/dtsf2025/uncategorized/week-3-algorithmic-pivots-torque-troubles-and-a-breakthrough/

Strummer + stepper setup:

- Rachel De Barros — stepper motor tutorial  
  https://www.youtube.com/watch?v=wcLeXXATCR4

Torque concept (static vs dynamic context):

- ScienceDirect Topics — static torque  
  https://www.sciencedirect.com/topics/engineering/static-torque

Related builds that helped me triangulate what’s possible:

- MECH 5840 (Auburn) — stepper-based tuner example (NEMA 23)  
  https://www.youtube.com/watch?v=DU2-K5PHwyM&t=147s
- Tim Bell — early automatic tuner inspiration  
  https://www.youtube.com/watch?v=ZahpROn3gm0

Project blog archive:

- DTSF project page (all weekly posts)  
  https://dtsf.sites.gettysburg.edu/dtsf2025/taha
