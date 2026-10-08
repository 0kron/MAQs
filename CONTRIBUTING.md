# Guía de colaboración

Esta guía presenta un flujo inicial para mantener el trabajo ordenado y 
permitir su revisión mediante Git y GitHub.

## Flujo general

1. **NUNCA** trabajes directamente sobre `main`.
2. Antes de comenzar, actualiza tu repositorio cuando corresponda.
3. Crea una rama para cada actividad o modificación.
4. Usa `git status` con frecuencia para conocer el estado de tus archivos.
5. Revisa los cambios con `git diff`.
6. Haz commits pequeños y descriptivos.
7. Abre un pull request cuando el trabajo esté listo para revisión.
8. Revisa tu propio trabajo antes de solicitar una revisión.
9. Responde las observaciones realizadas en el pull request.
10. Realiza revisión entre compañeros cuando la actividad lo indique.

## Ramas de trabajo

Las ramas de nuevos features o ideas siguen la siguiente convención:

```text
<nombre del feature>/<nombre del autor>
```

Utiliza minúsculas y guiones si así lo requiere el feature. Solo utiliza tu 
nombre de pila.

## Mensajes de commit

Usaremos una versión flexible de Conventional Commits para que los mensajes 
sean fáciles de comprender. Algunos ejemplos son:

```text
docs: agregar archivos varios
feat: agregar implementación
fix: corrigir feature o bug
refactor: reorganizar código
test: agregar pruebas
```

Cada commit debe respetar las convenciones y ser claro en su efecto.

## Pull requests

El pull request debe incluir un resumen, los archivos modificados, la manera de
ejecución y resultado de lo agregado en caso de aplicar. Así como dudas o 
dificultades. 

Revisa antes `git status` y `git diff`; no incluyas cachés, salidas temporales 
ni archivos específicos de tu entorno, ejemplo: `.venv.`. Solamente el dueño del
**repo** tiene permitido hacer merge.

El título del PR debe de ser: 
```
[equipo/feature] nombre (<dia,numero,mes> <hora 24 hrs>)
```
