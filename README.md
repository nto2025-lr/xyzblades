# xyzblades
Репозиторий команды "XYZBlades" на НТО 2025 по профилю "Летающая робототехника".

В этом репозитории в папках day1, day2 и day3 прикреплены файлы полетных миссий нашей команды. 
В каждой из папок на дни есть файл для полета - main.py, файл, содержащий часть бэкэнда для общения с фронтендом, и папки templates и static.
В templates лежит файл index.html, содержащий html-темплейт для фронтенда. В static - style.css, содержащий стили для всех элементов фронтенда.

Целевой алгоритм работы оставался одним и тем же на протяжении всех дней, менялись только способы его реализации, дорабатывались алгоритмы компьютерного зрения.
Порядок действий дрона на третьей зачетной попытке:
- поднятие сервера для веб-приложения
- получение с сервера данных и легальных, и нелегальных врезок
- запуск бесконечного цикла - работы основного алгоритма
- В нем - ожидание нажатия кнопки старт на веб-сервере
- далее - отправка сигнала на дронопорт об открытии крышек, ожидание ответа от дронопорта
- автономный взлет на высоту 1 м и отправка сигнала для закрытия дронопорта
- полет к началу участка теплосети
- запуск записи видео с тепловизора и основной камеры
- полет по координатам частей трубы с определением участков утечки тепла
- полет до окончания участка теплосети для завершения мониторинга
- определение начал и концов участков утечки
- отправка координат на веб-сервер для визуализации
- окончание записи видео и запись в файл.

- Далее выполняется полет на координаты в районе расположения нефтепровода для обеспечения стабильности и преодоления колебаний из-за погрешностей навигации
- Разворот вокруг оси Z по yaw для выравнивания по трубе
- полет по координатам нелегальных врезок
- фотографирование нелегальных врезок с их выделением
- возврат на старт и посадка.

Для поиска участков утечки тепла использовался поиск контуров по изображению с тепловизора и определение расстояния от контуров до центра изображения
Для выделения нелегальных врезок использовался поиск контуров, подходящих под маску по изображению с основной камеры, а также отсеивание контура центральной части через определение координат посредством замены части изображения черными пикселями


В папке "ИНЖЕНЕРНАЯ ЧАСТЬ" прикреплены все материалы по постройке и эксплуатации дронопорта: 3д-модели, чертежи, выполненнные по ГОСТ, инструкция по сборке и  эксплуатации устройства, а также в папке "ПРИЛОЖЕНИЯ" прикреплен весь программный код, необходимый для работы и тестирования устройства. В процессе работы самого дрона команды для открытия и закрытия дронопорта отправляются в автоматическом режиме через WebSocket.

Также в папку с инженерной частью прикреплено общее фото команды и видеозапись работы дронопорта.
