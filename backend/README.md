## Format script
```bash
find . -iname *.h -not -path "./build/*" -o -iname *.cc -not -path "./build/*" | xargs clang-format -i -style=file
```